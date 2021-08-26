/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.cpp - Camera HAL stream
 */

#include "camera_stream.h"

#include <sys/mman.h>

#include "camera_buffer.h"
#include "camera_device.h"
#include "camera_metadata.h"
#include "jpeg/post_processor_jpeg.h"

#include <libcamera/formats.h>

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

/*
 * \class CameraStream
 * \brief Map a camera3_stream_t to a StreamConfiguration
 *
 * The CameraStream class maps a camera3_stream_t provided by Android
 * camera framework to a libcamera::StreamConfiguration.
 *
 * The StreamConfiguration is represented by its index as recorded in the
 * CameraConfiguration and not by pointer as StreamConfiguration is subject to
 * relocation.
 *
 * A single StreamConfiguration may be used to deliver one or more streams to
 * the Android framework. The mapping type between a camera3 stream to a
 * StreamConfiguration is described by the CameraStream::Type.
 *
 * CameraStream handles all the aspects of producing a stream with the size
 * and format requested by the camera3 stream from the data produced by
 * the associated libcamera::Stream, including the creation of the encoder
 * and buffer allocation.
 */

CameraStream::CameraStream(CameraDevice *const cameraDevice,
			   CameraConfiguration *config, Type type,
			   camera3_stream_t *camera3Stream, unsigned int index)
	: cameraDevice_(cameraDevice), config_(config), type_(type),
	  camera3Stream_(camera3Stream), index_(index)
{
	if (type_ == Type::Internal || type_ == Type::Mapped) {
		/*
		 * \todo There might be multiple post-processors. The logic
		 * which should be instantiated here, is deferred for the
		 * future. For now, we only have PostProcessorJpeg and that
		 * is what we instantiate here.
		 */
		postProcessor_ = std::make_unique<PostProcessorJpeg>(cameraDevice_);
	}

	if (type == Type::Internal) {
		allocator_ = std::make_unique<FrameBufferAllocator>(cameraDevice_->camera());
		mutex_ = std::make_unique<std::mutex>();
	}
}

const StreamConfiguration &CameraStream::configuration() const
{
	return config_->at(index_);
}

Stream *CameraStream::stream() const
{
	return configuration().stream();
}

int CameraStream::configure()
{
	if (postProcessor_) {
		StreamConfiguration output = configuration();
		output.pixelFormat = formats::MJPEG;
		int ret = postProcessor_->configure(configuration(), output);
		if (ret)
			return ret;
	}

	if (allocator_) {
		int ret = allocator_->allocate(stream());
		if (ret < 0)
			return ret;

		/* Save a pointer to the reserved frame buffers */
		for (const auto &frameBuffer : allocator_->buffers(stream()))
			buffers_.push_back(frameBuffer.get());
	}

	camera3Stream_->max_buffers = configuration().bufferCount;

	return 0;
}

int CameraStream::process(const libcamera::FrameBuffer &source,
			  buffer_handle_t camera3Dest,
			  const CameraMetadata &requestMetadata,
			  CameraMetadata *resultMetadata)
{
	if (!postProcessor_)
		return 0;

	/*
	 * \todo Buffer mapping and processing should be moved to a
	 * separate thread.
	 */
	const StreamConfiguration &output = configuration();
	CameraBuffer dest(camera3Dest, formats::MJPEG, output.size,
			  PROT_READ | PROT_WRITE);
	if (!dest.isValid()) {
		LOG(HAL, Error) << "Failed to map android blob buffer";
		return -EINVAL;
	}

	return postProcessor_->process(source, &dest, requestMetadata, resultMetadata);
}

FrameBuffer *CameraStream::getBuffer()
{
	if (!allocator_)
		return nullptr;

	std::lock_guard<std::mutex> locker(*mutex_);

	if (buffers_.empty()) {
		LOG(HAL, Error) << "Buffer underrun";
		return nullptr;
	}

	FrameBuffer *buffer = buffers_.back();
	buffers_.pop_back();

	return buffer;
}

void CameraStream::putBuffer(libcamera::FrameBuffer *buffer)
{
	if (!allocator_)
		return;

	std::lock_guard<std::mutex> locker(*mutex_);

	buffers_.push_back(buffer);
}
