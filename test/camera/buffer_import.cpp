/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 *
 * Test importing buffers exported from the VIVID output device into a Camera
 */

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "buffer_source.h"
#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std::chrono_literals;

namespace {

class BufferImportTest : public CameraTest, public Test
{
public:
	BufferImportTest()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	void bufferComplete([[maybe_unused]] Request *request,
			    FrameBuffer *buffer)
	{
		if (buffer->metadata().status != FrameMetadata::FrameSuccess)
			return;

		completeBuffersCount_++;
	}

	void requestComplete(Request *request)
	{
		if (request->status() != Request::RequestComplete)
			return;

		const Request::BufferMap &buffers = request->buffers();

		completeRequestsCount_++;

		/* Create a new request. */
		const Stream *stream = buffers.begin()->first;
		FrameBuffer *buffer = buffers.begin()->second;

		request->reuse();
		request->addBuffer(stream, buffer);
		camera_->queueRequest(request);

		dispatcher_->interrupt();
	}

	int init() override
	{
		if (status_ != TestPass)
			return status_;

		config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!config_ || config_->size() != 1) {
			std::cout << "Failed to generate default configuration" << std::endl;
			return TestFail;
		}

		dispatcher_ = Thread::current()->eventDispatcher();

		return TestPass;
	}

	int run() override
	{
		StreamConfiguration &cfg = config_->at(0);

		if (camera_->acquire()) {
			std::cout << "Failed to acquire the camera" << std::endl;
			return TestFail;
		}

		if (camera_->configure(config_.get())) {
			std::cout << "Failed to set default configuration" << std::endl;
			return TestFail;
		}

		Stream *stream = cfg.stream();

		BufferSource source;
		int ret = source.allocate(cfg);
		if (ret != TestPass)
			return ret;

		for (const std::unique_ptr<FrameBuffer> &buffer : source.buffers()) {
			std::unique_ptr<Request> request = camera_->createRequest();
			if (!request) {
				std::cout << "Failed to create request" << std::endl;
				return TestFail;
			}

			if (request->addBuffer(stream, buffer.get())) {
				std::cout << "Failed to associating buffer with request" << std::endl;
				return TestFail;
			}

			requests_.push_back(std::move(request));
		}

		completeRequestsCount_ = 0;
		completeBuffersCount_ = 0;

		camera_->bufferCompleted.connect(this, &BufferImportTest::bufferComplete);
		camera_->requestCompleted.connect(this, &BufferImportTest::requestComplete);

		if (camera_->start()) {
			std::cout << "Failed to start camera" << std::endl;
			return TestFail;
		}

		for (std::unique_ptr<Request> &request : requests_) {
			if (camera_->queueRequest(request.get())) {
				std::cout << "Failed to queue request" << std::endl;
				return TestFail;
			}
		}

		const unsigned int nFrames = cfg.bufferCount * 2;

		Timer timer;
		timer.start(500ms * nFrames);
		while (timer.isRunning()) {
			dispatcher_->processEvents();
			if (completeRequestsCount_ > nFrames)
				break;
		}

		if (completeRequestsCount_ < nFrames) {
			std::cout << "Failed to capture enough frames (got "
				  << completeRequestsCount_ << " expected at least "
				  << nFrames << ")" << std::endl;
			return TestFail;
		}

		if (completeRequestsCount_ != completeBuffersCount_) {
			std::cout << "Number of completed buffers and requests differ" << std::endl;
			return TestFail;
		}

		if (camera_->stop()) {
			std::cout << "Failed to stop camera" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

private:
	EventDispatcher *dispatcher_;

	std::vector<std::unique_ptr<Request>> requests_;

	unsigned int completeBuffersCount_;
	unsigned int completeRequestsCount_;
	std::unique_ptr<CameraConfiguration> config_;
};

} /* namespace */

TEST_REGISTER(BufferImportTest)
