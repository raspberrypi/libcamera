/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include <libcamera/framebuffer_allocator.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

namespace {

class Capture : public CameraTest, public Test
{
public:
	Capture()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	unsigned int completeBuffersCount_;
	unsigned int completeRequestsCount_;

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
			cout << "Failed to generate default configuration" << endl;
			return TestFail;
		}

		allocator_ = new FrameBufferAllocator(camera_);
		dispatcher_ = Thread::current()->eventDispatcher();

		return TestPass;
	}

	void cleanup() override
	{
		delete allocator_;
	}

	int run() override
	{
		StreamConfiguration &cfg = config_->at(0);

		if (camera_->acquire()) {
			cout << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		if (camera_->configure(config_.get())) {
			cout << "Failed to set default configuration" << endl;
			return TestFail;
		}

		Stream *stream = cfg.stream();

		int ret = allocator_->allocate(stream);
		if (ret < 0)
			return TestFail;

		for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
			std::unique_ptr<Request> request = camera_->createRequest();
			if (!request) {
				cout << "Failed to create request" << endl;
				return TestFail;
			}

			if (request->addBuffer(stream, buffer.get())) {
				cout << "Failed to associate buffer with request" << endl;
				return TestFail;
			}

			requests_.push_back(std::move(request));
		}

		completeRequestsCount_ = 0;
		completeBuffersCount_ = 0;

		camera_->bufferCompleted.connect(this, &Capture::bufferComplete);
		camera_->requestCompleted.connect(this, &Capture::requestComplete);

		if (camera_->start()) {
			cout << "Failed to start camera" << endl;
			return TestFail;
		}

		for (std::unique_ptr<Request> &request : requests_) {
			if (camera_->queueRequest(request.get())) {
				cout << "Failed to queue request" << endl;
				return TestFail;
			}
		}

		unsigned int nFrames = allocator_->buffers(stream).size() * 2;

		Timer timer;
		timer.start(500ms * nFrames);
		while (timer.isRunning()) {
			dispatcher_->processEvents();
			if (completeRequestsCount_ > nFrames)
				break;
		}

		if (completeRequestsCount_ < nFrames) {
			cout << "Failed to capture enough frames (got "
			     << completeRequestsCount_ << " expected at least "
			     << nFrames * 2 << ")" << endl;
			return TestFail;
		}

		if (completeRequestsCount_ != completeBuffersCount_) {
			cout << "Number of completed buffers and requests differ" << endl;
			return TestFail;
		}

		if (camera_->stop()) {
			cout << "Failed to stop camera" << endl;
			return TestFail;
		}

		return TestPass;
	}

	EventDispatcher *dispatcher_;

	std::vector<std::unique_ptr<Request>> requests_;

	std::unique_ptr<CameraConfiguration> config_;
	FrameBufferAllocator *allocator_;
};

} /* namespace */

TEST_REGISTER(Capture)
