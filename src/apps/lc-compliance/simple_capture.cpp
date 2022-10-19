/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020-2021, Google Inc.
 *
 * simple_capture.cpp - Simple capture helper
 */

#include <gtest/gtest.h>

#include "simple_capture.h"

using namespace libcamera;

SimpleCapture::SimpleCapture(std::shared_ptr<Camera> camera)
	: loop_(nullptr), camera_(camera),
	  allocator_(std::make_unique<FrameBufferAllocator>(camera))
{
}

SimpleCapture::~SimpleCapture()
{
	stop();
}

void SimpleCapture::configure(StreamRole role)
{
	config_ = camera_->generateConfiguration({ role });

	if (!config_) {
		std::cout << "Role not supported by camera" << std::endl;
		GTEST_SKIP();
	}

	if (config_->validate() != CameraConfiguration::Valid) {
		config_.reset();
		FAIL() << "Configuration not valid";
	}

	if (camera_->configure(config_.get())) {
		config_.reset();
		FAIL() << "Failed to configure camera";
	}
}

void SimpleCapture::start()
{
	Stream *stream = config_->at(0).stream();
	int count = allocator_->allocate(stream);

	ASSERT_GE(count, 0) << "Failed to allocate buffers";
	EXPECT_EQ(count, config_->at(0).bufferCount) << "Allocated less buffers than expected";

	camera_->requestCompleted.connect(this, &SimpleCapture::requestComplete);

	ASSERT_EQ(camera_->start(), 0) << "Failed to start camera";
}

void SimpleCapture::stop()
{
	if (!config_ || !allocator_->allocated())
		return;

	camera_->stop();

	camera_->requestCompleted.disconnect(this);

	Stream *stream = config_->at(0).stream();
	allocator_->free(stream);
}

/* SimpleCaptureBalanced */

SimpleCaptureBalanced::SimpleCaptureBalanced(std::shared_ptr<Camera> camera)
	: SimpleCapture(camera)
{
}

void SimpleCaptureBalanced::capture(unsigned int numRequests)
{
	start();

	Stream *stream = config_->at(0).stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);

	/* No point in testing less requests then the camera depth. */
	if (buffers.size() > numRequests) {
		std::cout << "Camera needs " + std::to_string(buffers.size())
			+ " requests, can't test only "
			+ std::to_string(numRequests) << std::endl;
		GTEST_SKIP();
	}

	queueCount_ = 0;
	captureCount_ = 0;
	captureLimit_ = numRequests;

	/* Queue the recommended number of reqeuests. */
	std::vector<std::unique_ptr<libcamera::Request>> requests;
	for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
		std::unique_ptr<Request> request = camera_->createRequest();
		ASSERT_TRUE(request) << "Can't create request";

		ASSERT_EQ(request->addBuffer(stream, buffer.get()), 0) << "Can't set buffer for request";

		ASSERT_EQ(queueRequest(request.get()), 0) << "Failed to queue request";

		requests.push_back(std::move(request));
	}

	/* Run capture session. */
	loop_ = new EventLoop();
	loop_->exec();
	stop();
	delete loop_;

	ASSERT_EQ(captureCount_, captureLimit_);
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

/* SimpleCaptureUnbalanced */

SimpleCaptureUnbalanced::SimpleCaptureUnbalanced(std::shared_ptr<Camera> camera)
	: SimpleCapture(camera)
{
}

void SimpleCaptureUnbalanced::capture(unsigned int numRequests)
{
	start();

	Stream *stream = config_->at(0).stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);

	captureCount_ = 0;
	captureLimit_ = numRequests;

	/* Queue the recommended number of reqeuests. */
	std::vector<std::unique_ptr<libcamera::Request>> requests;
	for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
		std::unique_ptr<Request> request = camera_->createRequest();
		ASSERT_TRUE(request) << "Can't create request";

		ASSERT_EQ(request->addBuffer(stream, buffer.get()), 0) << "Can't set buffer for request";

		ASSERT_EQ(camera_->queueRequest(request.get()), 0) << "Failed to queue request";

		requests.push_back(std::move(request));
	}

	/* Run capture session. */
	loop_ = new EventLoop();
	int status = loop_->exec();
	stop();
	delete loop_;

	ASSERT_EQ(status, 0);
}

void SimpleCaptureUnbalanced::requestComplete(Request *request)
{
	captureCount_++;
	if (captureCount_ >= captureLimit_) {
		loop_->exit(0);
		return;
	}

	request->reuse(Request::ReuseBuffers);
	if (camera_->queueRequest(request))
		loop_->exit(-EINVAL);
}
