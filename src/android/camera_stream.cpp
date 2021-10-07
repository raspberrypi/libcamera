/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.cpp - Camera HAL stream
 */

#include "camera_stream.h"

#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <unistd.h>

#include <libcamera/formats.h>

#include "jpeg/post_processor_jpeg.h"
#include "yuv/post_processor_yuv.h"

#include "camera_buffer.h"
#include "camera_capabilities.h"
#include "camera_device.h"
#include "camera_metadata.h"

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
	if (type_ == Type::Internal || type_ == Type::Mapped) {
		const PixelFormat outFormat =
			cameraDevice_->capabilities()->toPixelFormat(camera3Stream_->format);
		StreamConfiguration output = configuration();
		output.pixelFormat = outFormat;
		output.size.width = camera3Stream_->width;
		output.size.height = camera3Stream_->height;

		switch (outFormat) {
		case formats::NV12:
			postProcessor_ = std::make_unique<PostProcessorYuv>();
			break;

		case formats::MJPEG:
			postProcessor_ = std::make_unique<PostProcessorJpeg>(cameraDevice_);
			break;

		default:
			LOG(HAL, Error) << "Unsupported format: " << outFormat;
			return -EINVAL;
		}

		int ret = postProcessor_->configure(configuration(), output);
		if (ret)
			return ret;
	}

	if (type_ == Type::Internal) {
		allocator_ = std::make_unique<FrameBufferAllocator>(cameraDevice_->camera());
		mutex_ = std::make_unique<std::mutex>();

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

int CameraStream::waitFence(int fence)
{
	/*
	 * \todo The implementation here is copied from camera_worker.cpp
	 * and both should be removed once libcamera is instrumented to handle
	 * fences waiting in the core.
	 *
	 * \todo Better characterize the timeout. Currently equal to the one
	 * used by the Rockchip Camera HAL on ChromeOS.
	 */
	constexpr unsigned int timeoutMs = 300;
	struct pollfd fds = { fence, POLLIN, 0 };

	do {
		int ret = poll(&fds, 1, timeoutMs);
		if (ret == 0)
			return -ETIME;

		if (ret > 0) {
			if (fds.revents & (POLLERR | POLLNVAL))
				return -EINVAL;

			return 0;
		}
	} while (errno == EINTR || errno == EAGAIN);

	return -errno;
}

int CameraStream::process(const FrameBuffer &source,
			  camera3_stream_buffer_t &camera3Dest,
			  const CameraMetadata &requestMetadata,
			  CameraMetadata *resultMetadata)
{
	/* Handle waiting on fences on the destination buffer. */
	int fence = camera3Dest.acquire_fence;
	if (fence != -1) {
		int ret = waitFence(fence);
		::close(fence);
		camera3Dest.acquire_fence = -1;
		if (ret < 0) {
			LOG(HAL, Error) << "Failed waiting for fence: "
					<< fence << ": " << strerror(-ret);
			return ret;
		}
	}

	if (!postProcessor_)
		return 0;

	/*
	 * \todo Buffer mapping and processing should be moved to a
	 * separate thread.
	 */
	const StreamConfiguration &output = configuration();
	CameraBuffer dest(*camera3Dest.buffer, output.pixelFormat, output.size,
			  PROT_READ | PROT_WRITE);
	if (!dest.isValid()) {
		LOG(HAL, Error) << "Failed to create destination buffer";
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

void CameraStream::putBuffer(FrameBuffer *buffer)
{
	if (!allocator_)
		return;

	std::lock_guard<std::mutex> locker(*mutex_);

	buffers_.push_back(buffer);
}
