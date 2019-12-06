/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_camera.cpp - V4L2 compatibility camera
 */

#include "v4l2_camera.h"

#include <errno.h>

#include "log.h"
#include "utils.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(V4L2Compat);

FrameMetadata::FrameMetadata(Buffer *buffer)
	: index_(buffer->index()), bytesused_(buffer->bytesused()),
	  timestamp_(buffer->timestamp()), sequence_(buffer->sequence()),
	  status_(buffer->status())
{
}

V4L2Camera::V4L2Camera(std::shared_ptr<Camera> camera)
	: camera_(camera), isRunning_(false)
{
	camera_->requestCompleted.connect(this, &V4L2Camera::requestComplete);
}

V4L2Camera::~V4L2Camera()
{
	camera_->release();
}

void V4L2Camera::open(int *ret)
{
	/* \todo Support multiple open. */
	if (camera_->acquire() < 0) {
		LOG(V4L2Compat, Error) << "Failed to acquire camera";
		*ret = -EINVAL;
		return;
	}

	config_ = camera_->generateConfiguration({ StreamRole::Viewfinder });
	if (!config_) {
		camera_->release();
		*ret = -EINVAL;
		return;
	}

	*ret = 0;
}

void V4L2Camera::close()
{
	camera_->release();
}

void V4L2Camera::getStreamConfig(StreamConfiguration *streamConfig)
{
	*streamConfig = config_->at(0);
}

std::vector<FrameMetadata> V4L2Camera::completedBuffers()
{
	std::vector<FrameMetadata> v;

	bufferLock_.lock();
	for (std::unique_ptr<FrameMetadata> &metadata : completedBuffers_)
		v.push_back(*metadata.get());
	completedBuffers_.clear();
	bufferLock_.unlock();

	return v;
}

void V4L2Camera::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	/* We only have one stream at the moment. */
	bufferLock_.lock();
	Buffer *buffer = request->buffers().begin()->second;
	std::unique_ptr<FrameMetadata> metadata =
		utils::make_unique<FrameMetadata>(buffer);
	completedBuffers_.push_back(std::move(metadata));
	bufferLock_.unlock();

	bufferSema_.release();
}

void V4L2Camera::configure(int *ret, StreamConfiguration *streamConfigOut,
			   const Size &size, PixelFormat pixelformat,
			   unsigned int bufferCount)
{
	StreamConfiguration &streamConfig = config_->at(0);
	streamConfig.size.width = size.width;
	streamConfig.size.height = size.height;
	streamConfig.pixelFormat = pixelformat;
	streamConfig.bufferCount = bufferCount;
	/* \todo memoryType (interval vs external) */

	CameraConfiguration::Status validation = config_->validate();
	if (validation == CameraConfiguration::Invalid) {
		LOG(V4L2Compat, Debug) << "Configuration invalid";
		*ret = -EINVAL;
		return;
	}
	if (validation == CameraConfiguration::Adjusted)
		LOG(V4L2Compat, Debug) << "Configuration adjusted";

	LOG(V4L2Compat, Debug) << "Validated configuration is: "
			      << streamConfig.toString();

	*ret = camera_->configure(config_.get());
	if (*ret < 0)
		return;

	*streamConfigOut = config_->at(0);
}

void V4L2Camera::mmap(void **ret, unsigned int index)
{
	Stream *stream = *camera_->streams().begin();
	*ret = stream->buffers()[index].planes()[0].mem();
}

void V4L2Camera::allocBuffers(int *ret, unsigned int count)
{
	*ret = camera_->allocateBuffers();
	if (*ret == -EACCES)
		*ret = -EBUSY;
}

void V4L2Camera::freeBuffers()
{
	camera_->freeBuffers();
}

void V4L2Camera::streamOn(int *ret)
{
	*ret = 0;

	if (isRunning_)
		return;

	*ret = camera_->start();
	if (*ret < 0) {
		if (*ret == -EACCES)
			*ret = -EBUSY;
		return;
	}
	isRunning_ = true;

	for (std::unique_ptr<Request> &req : pendingRequests_) {
		/* \todo What should we do if this returns -EINVAL? */
		*ret = camera_->queueRequest(req.release());
		if (*ret < 0) {
			if (*ret == -EACCES)
				*ret = -EBUSY;
			return;
		}
	}

	pendingRequests_.clear();
}

void V4L2Camera::streamOff(int *ret)
{
	*ret = 0;

	/* \todo Restore buffers to reqbufs state? */
	if (!isRunning_)
		return;

	*ret = camera_->stop();
	if (*ret < 0) {
		if (*ret == -EACCES)
			*ret = -EBUSY;
		return;
	}
	isRunning_ = false;
}

void V4L2Camera::qbuf(int *ret, unsigned int index)
{
	Stream *stream = config_->at(0).stream();
	std::unique_ptr<Buffer> buffer = stream->createBuffer(index);
	if (!buffer) {
		LOG(V4L2Compat, Error) << "Can't create buffer";
		*ret = -ENOMEM;
		return;
	}

	std::unique_ptr<Request> request =
		std::unique_ptr<Request>(camera_->createRequest());
	if (!request) {
		LOG(V4L2Compat, Error) << "Can't create request";
		*ret = -ENOMEM;
		return;
	}

	*ret = request->addBuffer(std::move(buffer));
	if (*ret < 0) {
		LOG(V4L2Compat, Error) << "Can't set buffer for request";
		*ret = -ENOMEM;
		return;
	}

	if (!isRunning_) {
		pendingRequests_.push_back(std::move(request));
		return;
	}

	*ret = camera_->queueRequest(request.release());
	if (*ret < 0) {
		LOG(V4L2Compat, Error) << "Can't queue request";
		if (*ret == -EACCES)
			*ret = -EBUSY;
	}
}
