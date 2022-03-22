/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * fence.cpp - Fence test
 */

#include <iostream>
#include <memory>
#include <sys/eventfd.h>
#include <unistd.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>
#include <libcamera/base/unique_fd.h>
#include <libcamera/base/utils.h>

#include <libcamera/fence.h>
#include <libcamera/framebuffer_allocator.h>

#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

class FenceTest : public CameraTest, public Test
{
public:
	FenceTest();

protected:
	int init() override;
	int run() override;

private:
	int validateExpiredRequest(Request *request);
	int validateRequest(Request *request);
	void requestComplete(Request *request);
	void requestRequeue(Request *request);

	void signalFence();

	std::unique_ptr<Fence> fence_;
	EventDispatcher *dispatcher_;
	UniqueFD eventFd_;
	UniqueFD eventFd2_;
	Timer fenceTimer_;

	std::vector<std::unique_ptr<Request>> requests_;
	std::unique_ptr<CameraConfiguration> config_;
	std::unique_ptr<FrameBufferAllocator> allocator_;

	Stream *stream_;

	bool expectedCompletionResult_ = true;
	bool setFence_ = true;

	unsigned int completedRequest_;

	unsigned int signalledRequestId_;
	unsigned int expiredRequestId_;
	unsigned int nbuffers_;

	int efd2_;
	int efd_;
};

FenceTest::FenceTest()
	: CameraTest("platform/vimc.0 Sensor B")
{
}

int FenceTest::init()
{
	/* Make sure the CameraTest constructor succeeded. */
	if (status_ != TestPass)
		return status_;

	dispatcher_ = Thread::current()->eventDispatcher();

	/*
	 * Create two eventfds to model the fences. This is enough to support the
	 * needs of libcamera which only needs to wait for read events through
	 * poll(). Once native support for fences will be available in the
	 * backend kernel APIs this will need to be replaced by a sw_sync fence,
	 * but that requires debugfs.
	 */
	eventFd_ = UniqueFD(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK));
	eventFd2_ = UniqueFD(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK));
	if (!eventFd_.isValid() || !eventFd2_.isValid()) {
		cerr << "Unable to create eventfd" << endl;
		return TestFail;
	}

	efd_ = eventFd_.get();
	efd2_ = eventFd2_.get();

	config_ = camera_->generateConfiguration({ StreamRole::Viewfinder });
	if (!config_ || config_->size() != 1) {
		cerr << "Failed to generate default configuration" << endl;
		return TestFail;
	}

	if (camera_->acquire()) {
		cerr << "Failed to acquire the camera" << endl;
		return TestFail;
	}

	if (camera_->configure(config_.get())) {
		cerr << "Failed to set default configuration" << endl;
		return TestFail;
	}

	StreamConfiguration &cfg = config_->at(0);
	stream_ = cfg.stream();

	allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
	if (allocator_->allocate(stream_) < 0)
		return TestFail;

	nbuffers_ = allocator_->buffers(stream_).size();
	if (nbuffers_ < 2) {
		cerr << "Not enough buffers available" << endl;
		return TestFail;
	}

	signalledRequestId_ = nbuffers_ - 2;
	expiredRequestId_ = nbuffers_ - 1;

	return TestPass;
}

int FenceTest::validateExpiredRequest(Request *request)
{
	/* The last request is expected to fail. */
	if (request->status() != Request::RequestCancelled) {
		cerr << "The last request should have failed: " << endl;
		return TestFail;
	}

	FrameBuffer *buffer = request->buffers().begin()->second;
	std::unique_ptr<Fence> fence = buffer->releaseFence();
	if (!fence) {
		cerr << "The expired fence should be present" << endl;
		return TestFail;
	}

	if (!fence->isValid()) {
		cerr << "The expired fence should be valid" << endl;
		return TestFail;
	}

	UniqueFD fd = fence->release();
	if (fd.get() != efd_) {
		cerr << "The expired fence file descriptor should not change" << endl;
		return TestFail;
	}

	return TestPass;
}

int FenceTest::validateRequest(Request *request)
{
	uint64_t cookie = request->cookie();

	/* All requests but the last are expected to succeed. */
	if (request->status() != Request::RequestComplete) {
		cerr << "Unexpected request failure: " << cookie << endl;
		return TestFail;
	}

	/* A successfully completed request should have the Fence closed. */
	const Request::BufferMap &buffers = request->buffers();
	FrameBuffer *buffer = buffers.begin()->second;

	std::unique_ptr<Fence> bufferFence = buffer->releaseFence();
	if (bufferFence) {
		cerr << "Unexpected valid fence in completed request" << endl;
		return TestFail;
	}

	return TestPass;
}

void FenceTest::requestRequeue(Request *request)
{
	const Request::BufferMap &buffers = request->buffers();
	const Stream *stream = buffers.begin()->first;
	FrameBuffer *buffer = buffers.begin()->second;
	uint64_t cookie = request->cookie();

	request->reuse();

	if (cookie == signalledRequestId_ && setFence_) {
		/*
		 * The second time this request is queued add a fence to it.
		 *
		 * The main loop signals it by using a timer to write to the
		 * efd2_ file descriptor before the fence expires.
		 */
		std::unique_ptr<Fence> fence =
			std::make_unique<Fence>(std::move(eventFd2_));
		request->addBuffer(stream, buffer, std::move(fence));
	} else {
		/* All the other requests continue to operate without fences. */
		request->addBuffer(stream, buffer);
	}

	camera_->queueRequest(request);
}

void FenceTest::requestComplete(Request *request)
{
	uint64_t cookie = request->cookie();
	completedRequest_ = cookie;

	/*
	 * The last request is expected to fail as its fence has not been
	 * signaled.
	 *
	 * Validate the fence status but do not re-queue it.
	 */
	if (cookie == expiredRequestId_) {
		if (validateExpiredRequest(request) != TestPass)
			expectedCompletionResult_ = false;

		dispatcher_->interrupt();
		return;
	}

	/* Validate all requests but the last. */
	if (validateRequest(request) != TestPass) {
		expectedCompletionResult_ = false;

		dispatcher_->interrupt();
		return;
	}

	requestRequeue(request);

	/*
	 * Interrupt the dispatcher to return control to the main loop and
	 * activate the fenceTimer.
	 */
	dispatcher_->interrupt();
}

/* Callback to signal a fence waiting on the eventfd file descriptor. */
void FenceTest::signalFence()
{
	uint64_t value = 1;
	int ret;

	ret = write(efd2_, &value, sizeof(value));
	if (ret != sizeof(value))
		cerr << "Failed to signal fence" << endl;

	setFence_ = false;
	dispatcher_->processEvents();
}

int FenceTest::run()
{
	for (const auto &[i, buffer] : utils::enumerate(allocator_->buffers(stream_))) {
		std::unique_ptr<Request> request = camera_->createRequest(i);
		if (!request) {
			cerr << "Failed to create request" << endl;
			return TestFail;
		}

		int ret;
		if (i == expiredRequestId_) {
			/* This request will have a fence, and it will expire. */
			fence_ = std::make_unique<Fence>(std::move(eventFd_));
			if (!fence_->isValid()) {
				cerr << "Fence should be valid" << endl;
				return TestFail;
			}

			ret = request->addBuffer(stream_, buffer.get(), std::move(fence_));
		} else {
			/* All other requests will have no Fence. */
			ret = request->addBuffer(stream_, buffer.get());
		}

		if (ret) {
			cerr << "Failed to associate buffer with request" << endl;
			return TestFail;
		}

		requests_.push_back(std::move(request));
	}

	camera_->requestCompleted.connect(this, &FenceTest::requestComplete);

	if (camera_->start()) {
		cerr << "Failed to start camera" << endl;
		return TestFail;
	}

	for (std::unique_ptr<Request> &request : requests_) {
		if (camera_->queueRequest(request.get())) {
			cerr << "Failed to queue request" << endl;
			return TestFail;
		}
	}

	expectedCompletionResult_ = true;

	/* This timer serves to signal fences associated with "signalledRequestId_" */
	Timer fenceTimer;
	fenceTimer.timeout.connect(this, &FenceTest::signalFence);

	/* Loop for one second. */
	Timer timer;
	timer.start(1000ms);
	while (timer.isRunning() && expectedCompletionResult_) {
		if (completedRequest_ == signalledRequestId_ && setFence_)
			/*
			 * signalledRequestId_ has just completed and it has
			 * been re-queued with a fence. Start the timer to
			 * signal the fence in 10 msec.
			 */
			fenceTimer.start(10ms);

		dispatcher_->processEvents();
	}

	camera_->requestCompleted.disconnect();

	if (camera_->stop()) {
		cerr << "Failed to stop camera" << endl;
		return TestFail;
	}

	return expectedCompletionResult_ ? TestPass : TestFail;
}

TEST_REGISTER(FenceTest)
