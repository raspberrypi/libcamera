/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * framebuffer_allocator.cpp - FrameBuffer allocator
 */

#include <libcamera/framebuffer_allocator.h>

#include <errno.h>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/stream.h>

#include "log.h"
#include "pipeline_handler.h"

/**
 * \file framebuffer_allocator.h
 * \brief FrameBuffer allocator
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Allocator)

/**
 * \class FrameBufferAllocator
 * \brief FrameBuffer allocator for applications
 *
 * The libcamera API is designed to consume buffers provided by applications as
 * FrameBuffer instances. This makes libcamera a user of buffers exported by
 * other devices (such as displays or video encoders), or allocated from an
 * external allocator (such as ION on Android platforms). In some situations,
 * applications do not have any mean to allocate or get hold of suitable
 * buffers, for instance when no other device is involved, on Linux platforms
 * that lack a centralized allocator. The FrameBufferAllocator class provides a
 * buffer allocator that can be used in these situations.
 *
 * Applications create a framebuffer allocator for a Camera, and use it to
 * allocate buffers for streams of a CameraConfiguration with allocate(). They
 * control which streams to allocate buffers for, and can thus use external
 * buffers for a subset of the streams if desired.
 *
 * Buffers are deleted for a stream with free(), and destroying the allocator
 * automatically deletes all allocated buffers. Applications own the buffers
 * allocated by the FrameBufferAllocator and are responsible for ensuring the
 * buffers are not deleted while they are in use (part of a Request that has
 * been queued and hasn't completed yet).
 *
 * Usage of the FrameBufferAllocator is optional, if all buffers for a camera
 * are provided externally applications shall not use this class.
 */

/**
 * \brief Create a FrameBuffer allocator
 * \param[in] camera The camera the allocator serves
 *
 * A single allocator may be created for a Camera instance.
 *
 * The caller is responsible for deleting the allocator before the camera is
 * released.
 *
 * \return A pointer to the newly created allocator object or nullptr on error
 */
FrameBufferAllocator *
FrameBufferAllocator::create(std::shared_ptr<Camera> camera)
{
	if (camera->allocator_) {
		LOG(Allocator, Error) << "Camera already has an allocator";
		return nullptr;
	}

	camera->allocator_ = new FrameBufferAllocator(camera);
	return camera->allocator_;
}

/**
 * \brief Construct a FrameBufferAllocator serving a camera
 * \param[in] camera The camera
 */
FrameBufferAllocator::FrameBufferAllocator(std::shared_ptr<Camera> camera)
	: camera_(camera)
{
}

FrameBufferAllocator::~FrameBufferAllocator()
{
	for (auto &value : buffers_) {
		Stream *stream = value.first;
		camera_->pipe_->freeFrameBuffers(camera_.get(), stream);
	}

	buffers_.clear();

	camera_->allocator_ = nullptr;
}

/**
 * \brief Allocate buffers for a configured stream
 * \param[in] stream The stream to allocate buffers for
 *
 * Allocate buffers suitable for capturing frames from the \a stream. The Camera
 * shall have been previously configured with Camera::configure() and shall be
 * stopped, and the stream shall be part of the active camera configuration.
 *
 * Upon successful allocation, the allocated buffers can be retrieved with the
 * buffers() method.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EACCES The camera is not in a state where buffers can be allocated
 * \retval -EINVAL The \a stream does not belong to the camera or the stream is
 * not part of the active camera configuration
 * \retval -EBUSY Buffers are already allocated for the \a stream
 */
int FrameBufferAllocator::allocate(Stream *stream)
{
	if (camera_->state_ != Camera::CameraConfigured &&
	    camera_->state_ != Camera::CameraPrepared) {
		LOG(Allocator, Error)
			<< "Camera must be in the configured state to allocate buffers";
		return -EACCES;
	}

	if (camera_->streams().find(stream) == camera_->streams().end()) {
		LOG(Allocator, Error)
			<< "Stream does not belong to " << camera_->name();
		return -EINVAL;
	}

	if (camera_->activeStreams_.find(stream) == camera_->activeStreams_.end()) {
		LOG(Allocator, Error)
			<< "Stream is not part of " << camera_->name()
			<< " active configuration";
		return -EINVAL;
	}

	if (buffers_.count(stream)) {
		LOG(Allocator, Error) << "Buffers already allocated for stream";
		return -EBUSY;
	}

	int ret = camera_->pipe_->exportFrameBuffers(camera_.get(), stream,
						     &buffers_[stream]);
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Free buffers previously allocated for a \a stream
 * \param[in] stream The stream
 *
 * Free buffers allocated with allocate().
 *
 * This invalidates the buffers returned by buffers().
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EACCES The camera is not in a state where buffers can be freed
 * \retval -EINVAL The allocator do not handle the \a stream
 */
int FrameBufferAllocator::free(Stream *stream)
{
	if (camera_->state_ != Camera::CameraConfigured && camera_->state_ != Camera::CameraPrepared) {
		LOG(Allocator, Error)
			<< "Camera must be in the configured state to free buffers";
		return -EACCES;
	}

	auto iter = buffers_.find(stream);
	if (iter == buffers_.end())
		return -EINVAL;

	std::vector<std::unique_ptr<FrameBuffer>> &buffers = iter->second;

	buffers.clear();

	camera_->pipe_->freeFrameBuffers(camera_.get(), stream);

	buffers_.erase(iter);

	return 0;
}

/**
 * \fn FrameBufferAllocator::allocated()
 * \brief Check if the allocator has allocated buffers for any stream
 * \return True if the allocator has allocated buffers for one or more
 * streams, false otherwise
 */

/**
 * \brief Retrieve the buffers allocated for a \a stream
 * \param[in] stream The stream to retrive buffers for
 *
 * This method shall only be called after successfully allocating buffers for
 * \a stream with allocate(). The returned buffers are valid until free() is
 * called for the same stream or the FrameBufferAllocator instance is destroyed.
 *
 * \return The buffers allocated for the \a stream
 */
const std::vector<std::unique_ptr<FrameBuffer>> &
FrameBufferAllocator::buffers(Stream *stream) const
{
	static std::vector<std::unique_ptr<FrameBuffer>> empty;

	auto iter = buffers_.find(stream);
	if (iter == buffers_.end())
		return empty;

	return iter->second;
}

} /* namespace libcamera */
