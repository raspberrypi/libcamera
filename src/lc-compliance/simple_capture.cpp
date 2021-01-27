/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * simple_capture.cpp - Simple capture helper
 */

#include "simple_capture.h"

using namespace libcamera;

SimpleCapture::SimpleCapture(std::shared_ptr<Camera> camera)
	: camera_(camera), allocator_(std::make_unique<FrameBufferAllocator>(camera))
{
}

SimpleCapture::~SimpleCapture()
{
}

Results::Result SimpleCapture::configure(StreamRole role)
{
	config_ = camera_->generateConfiguration({ role });

	if (config_->validate() != CameraConfiguration::Valid) {
		config_.reset();
		return { Results::Fail, "Configuration not valid" };
	}

	if (camera_->configure(config_.get())) {
		config_.reset();
		return { Results::Fail, "Failed to configure camera" };
	}

	return { Results::Pass, "Configure camera" };
}

Results::Result SimpleCapture::start()
{
	Stream *stream = config_->at(0).stream();
	if (allocator_->allocate(stream) < 0)
		return { Results::Fail, "Failed to allocate buffers" };

	if (camera_->start())
		return { Results::Fail, "Failed to start camera" };

	camera_->requestCompleted.connect(this, &SimpleCapture::requestComplete);

	return { Results::Pass, "Started camera" };
}

Results::Result SimpleCapture::stop()
{
	Stream *stream = config_->at(0).stream();

	camera_->stop();

	camera_->requestCompleted.disconnect(this, &SimpleCapture::requestComplete);

	allocator_->free(stream);

	return { Results::Pass, "Stopped camera" };
}

/* SimpleCaptureBalanced */

SimpleCaptureBalanced::SimpleCaptureBalanced(std::shared_ptr<Camera> camera)
	: SimpleCapture(camera)
{
}

Results::Result SimpleCaptureBalanced::capture(unsigned int numRequests)
{
	Results::Result ret = start();
	if (ret.first != Results::Pass)
		return ret;

	Stream *stream = config_->at(0).stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);

	/* No point in testing less requests then the camera depth. */
	if (buffers.size() > numRequests) {
		/* Cache buffers.size() before we destroy it in stop() */
		int buffers_size = buffers.size();
		stop();

		return { Results::Skip, "Camera needs " + std::to_string(buffers_size)
			+ " requests, can't test only " + std::to_string(numRequests) };
	}

	queueCount_ = 0;
	captureCount_ = 0;
	captureLimit_ = numRequests;

	/* Queue the recommended number of reqeuests. */
	std::vector<std::unique_ptr<libcamera::Request>> requests;
	for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request) {
			stop();
			return { Results::Fail, "Can't create request" };
		}

		if (request->addBuffer(stream, buffer.get())) {
			stop();
			return { Results::Fail, "Can't set buffer for request" };
		}

		if (queueRequest(request.get()) < 0) {
			stop();
			return { Results::Fail, "Failed to queue request" };
		}

		requests.push_back(std::move(request));
	}

	/* Run capture session. */
	loop_ = new EventLoop();
	loop_->exec();
	stop();
	delete loop_;

	if (captureCount_ != captureLimit_)
		return { Results::Fail, "Got " + std::to_string(captureCount_) +
			" request, wanted " + std::to_string(captureLimit_) };

	return { Results::Pass, "Balanced capture of " +
		std::to_string(numRequests) + " requests" };
}

int SimpleCaptureBalanced::queueRequest(Request *request)
{
	queueCount_++;
	if (queueCount_ > captureLimit_)
		return 0;

	return camera_->queueRequest(request);
}

void SimpleCaptureBalanced::requestComplete(Request *request)
{
	captureCount_++;
	if (captureCount_ >= captureLimit_) {
		loop_->exit(0);
		return;
	}

	request->reuse(Request::ReuseBuffers);
	if (queueRequest(request))
		loop_->exit(-EINVAL);
}
