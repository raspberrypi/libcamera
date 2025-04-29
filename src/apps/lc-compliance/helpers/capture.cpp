/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020-2021, Google Inc.
 *
 * Simple capture helper
 */

#include "capture.h"

#include <assert.h>

#include <gtest/gtest.h>

using namespace libcamera;

Capture::Capture(std::shared_ptr<Camera> camera)
	: camera_(std::move(camera)), allocator_(camera_)
{
}

Capture::~Capture()
{
	stop();
}

void Capture::configure(libcamera::Span<const libcamera::StreamRole> roles)
{
	assert(!roles.empty());

	config_ = camera_->generateConfiguration(roles);
	if (!config_)
		GTEST_SKIP() << "Roles not supported by camera";

	ASSERT_EQ(config_->size(), roles.size()) << "Unexpected number of streams in configuration";

	/*
	 * Set the buffers count to the largest value across all streams.
	 * \todo: Should all streams from a Camera have the same buffer count ?
	 */
	auto largest =
		std::max_element(config_->begin(), config_->end(),
				 [](const StreamConfiguration &l, const StreamConfiguration &r)
				 { return l.bufferCount < r.bufferCount; });

	assert(largest != config_->end());

	for (auto &cfg : *config_)
		cfg.bufferCount = largest->bufferCount;

	if (config_->validate() != CameraConfiguration::Valid) {
		config_.reset();
		FAIL() << "Configuration not valid";
	}

	if (camera_->configure(config_.get())) {
		config_.reset();
		FAIL() << "Failed to configure camera";
	}
}

void Capture::run(unsigned int captureLimit, std::optional<unsigned int> queueLimit)
{
	assert(!queueLimit || captureLimit <= *queueLimit);

	captureLimit_ = captureLimit;
	queueLimit_ = queueLimit;

	captureCount_ = queueCount_ = 0;

	EventLoop loop;
	loop_ = &loop;

	start();

	for (const auto &request : requests_)
		queueRequest(request.get());

	EXPECT_EQ(loop_->exec(), 0);

	stop();

	EXPECT_LE(captureLimit_, captureCount_);
	EXPECT_LE(captureCount_, queueCount_);
	EXPECT_TRUE(!queueLimit_ || queueCount_ <= *queueLimit_);
}

int Capture::queueRequest(libcamera::Request *request)
{
	if (queueLimit_ && queueCount_ >= *queueLimit_)
		return 0;

	int ret = camera_->queueRequest(request);
	if (ret < 0)
		return ret;

	queueCount_ += 1;
	return 0;
}

void Capture::requestComplete(Request *request)
{
	captureCount_++;
	if (captureCount_ >= captureLimit_) {
		loop_->exit(0);
		return;
	}

	EXPECT_EQ(request->status(), Request::Status::RequestComplete)
		<< "Request didn't complete successfully";

	request->reuse(Request::ReuseBuffers);
	if (queueRequest(request))
		loop_->exit(-EINVAL);
}

void Capture::start()
{
	assert(config_);
	assert(!config_->empty());
	assert(!allocator_.allocated());
	assert(requests_.empty());

	const auto bufferCount = config_->at(0).bufferCount;

	/* No point in testing less requests then the camera depth. */
	if (queueLimit_ && *queueLimit_ < bufferCount) {
		GTEST_SKIP() << "Camera needs " << bufferCount
			     << " requests, can't test only " << *queueLimit_;
	}

	for (std::size_t i = 0; i < bufferCount; i++) {
		std::unique_ptr<Request> request = camera_->createRequest();
		ASSERT_TRUE(request) << "Can't create request";
		requests_.push_back(std::move(request));
	}

	for (const auto &cfg : *config_) {
		Stream *stream = cfg.stream();

		int count = allocator_.allocate(stream);
		ASSERT_GE(count, 0) << "Failed to allocate buffers";

		const auto &buffers = allocator_.buffers(stream);
		ASSERT_EQ(buffers.size(), bufferCount) << "Mismatching buffer count";

		for (std::size_t i = 0; i < bufferCount; i++) {
			ASSERT_EQ(requests_[i]->addBuffer(stream, buffers[i].get()), 0)
				<< "Failed to add buffer to request";
		}
	}

	ASSERT_TRUE(allocator_.allocated());

	camera_->requestCompleted.connect(this, &Capture::requestComplete);

	ASSERT_EQ(camera_->start(), 0) << "Failed to start camera";
}

void Capture::stop()
{
	if (!config_ || !allocator_.allocated())
		return;

	camera_->stop();

	camera_->requestCompleted.disconnect(this);

	requests_.clear();

	for (const auto &cfg : *config_) {
		EXPECT_EQ(allocator_.free(cfg.stream()), 0)
			<< "Failed to free buffers associated with stream";
	}

	EXPECT_FALSE(allocator_.allocated());
}
