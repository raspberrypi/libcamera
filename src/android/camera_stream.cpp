/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Camera HAL stream
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
#include "frame_buffer_allocator.h"
#include "post_processor.h"

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
			   camera3_stream_t *camera3Stream,
			   CameraStream *const sourceStream, unsigned int index)
	: cameraDevice_(cameraDevice), config_(config), type_(type),
	  camera3Stream_(camera3Stream), sourceStream_(sourceStream),
	  index_(index)
{
}

CameraStream::CameraStream(CameraStream &&other) = default;

CameraStream::~CameraStream()
{
	/*
	 * Manually delete buffers and then the allocator to make sure buffers
	 * are released while the allocator is still valid.
	 */
	allocatedBuffers_.clear();
	allocator_.reset();
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

		worker_ = std::make_unique<PostProcessorWorker>(postProcessor_.get());
		postProcessor_->processComplete.connect(
			this, [&](Camera3RequestDescriptor::StreamBuffer *streamBuffer,
				  PostProcessor::Status status) {
				Camera3RequestDescriptor::Status bufferStatus;

				if (status == PostProcessor::Status::Success)
					bufferStatus = Camera3RequestDescriptor::Status::Success;
				else
					bufferStatus = Camera3RequestDescriptor::Status::Error;

				cameraDevice_->streamProcessingComplete(streamBuffer,
									bufferStatus);
			});

		worker_->start();
	}

	allocator_ = std::make_unique<PlatformFrameBufferAllocator>(cameraDevice_);
	mutex_ = std::make_unique<Mutex>();

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

int CameraStream::process(Camera3RequestDescriptor::StreamBuffer *streamBuffer)
{
	ASSERT(type_ != Type::Direct);

	/* Handle waiting on fences on the destination buffer. */
	if (streamBuffer->fence.isValid()) {
		int ret = waitFence(streamBuffer->fence.get());
		if (ret < 0) {
			LOG(HAL, Error) << "Failed waiting for fence: "
					<< streamBuffer->fence.get() << ": "
					<< strerror(-ret);
			return ret;
		}

		streamBuffer->fence.reset();
	}

	const StreamConfiguration &output = configuration();
	streamBuffer->dstBuffer = std::make_unique<CameraBuffer>(
		*streamBuffer->camera3Buffer, output.pixelFormat, output.size,
		PROT_READ | PROT_WRITE);
	if (!streamBuffer->dstBuffer->isValid()) {
		LOG(HAL, Error) << "Failed to create destination buffer";
		return -EINVAL;
	}

	worker_->queueRequest(streamBuffer);

	return 0;
}

void CameraStream::flush()
{
	if (!postProcessor_)
		return;

	worker_->flush();
}

FrameBuffer *CameraStream::getBuffer()
{
	if (!allocator_)
		return nullptr;

	MutexLocker locker(*mutex_);

	if (buffers_.empty()) {
		/*
		 * Use HAL_PIXEL_FORMAT_YCBCR_420_888 unconditionally.
		 *
		 * YCBCR_420 is the source format for both the JPEG and the YUV
		 * post-processors.
		 *
		 * \todo Store a reference to the format of the source stream
		 * instead of hardcoding.
		 */
		auto frameBuffer = allocator_->allocate(HAL_PIXEL_FORMAT_YCBCR_420_888,
							configuration().size,
							camera3Stream_->usage);
		allocatedBuffers_.push_back(std::move(frameBuffer));
		buffers_.emplace_back(allocatedBuffers_.back().get());
	}

	FrameBuffer *buffer = buffers_.back();
	buffers_.pop_back();

	return buffer;
}

void CameraStream::putBuffer(FrameBuffer *buffer)
{
	if (!allocator_)
		return;

	MutexLocker locker(*mutex_);

	buffers_.push_back(buffer);
}

/**
 * \class CameraStream::PostProcessorWorker
 * \brief Post-process a CameraStream in an internal thread
 *
 * If the association between CameraStream and camera3_stream_t dictated by
 * CameraStream::Type is internal or mapped, the stream is generated by post
 * processing of a libcamera stream. Such a request is queued to a
 * PostProcessorWorker in CameraStream::process(). A queue of post-processing
 * requests is maintained by the PostProcessorWorker and it will run the
 * post-processing on an internal thread as soon as any request is available on
 * its queue.
 */
CameraStream::PostProcessorWorker::PostProcessorWorker(PostProcessor *postProcessor)
	: postProcessor_(postProcessor)
{
}

CameraStream::PostProcessorWorker::~PostProcessorWorker()
{
	{
		MutexLocker lock(mutex_);
		state_ = State::Stopped;
	}

	cv_.notify_one();
	wait();
}

void CameraStream::PostProcessorWorker::start()
{
	{
		MutexLocker lock(mutex_);
		ASSERT(state_ != State::Running);
		state_ = State::Running;
	}

	Thread::start();
}

void CameraStream::PostProcessorWorker::queueRequest(Camera3RequestDescriptor::StreamBuffer *dest)
{
	{
		MutexLocker lock(mutex_);
		ASSERT(state_ == State::Running);
		requests_.push(dest);
	}

	cv_.notify_one();
}

void CameraStream::PostProcessorWorker::run()
{
	MutexLocker locker(mutex_);

	while (1) {
		cv_.wait(locker, [&]() LIBCAMERA_TSA_REQUIRES(mutex_) {
			return state_ != State::Running || !requests_.empty();
		});

		if (state_ != State::Running)
			break;

		Camera3RequestDescriptor::StreamBuffer *streamBuffer = requests_.front();
		requests_.pop();
		locker.unlock();

		postProcessor_->process(streamBuffer);

		locker.lock();
	}

	if (state_ == State::Flushing) {
		std::queue<Camera3RequestDescriptor::StreamBuffer *> requests =
			std::move(requests_);
		locker.unlock();

		while (!requests.empty()) {
			postProcessor_->processComplete.emit(
				requests.front(), PostProcessor::Status::Error);
			requests.pop();
		}

		locker.lock();
		state_ = State::Stopped;
	}
}

void CameraStream::PostProcessorWorker::flush()
{
	MutexLocker lock(mutex_);
	state_ = State::Flushing;
	lock.unlock();

	cv_.notify_one();
}
