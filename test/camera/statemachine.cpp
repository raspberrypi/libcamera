/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include <libcamera/framebuffer_allocator.h>

#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std;

namespace {

class Statemachine : public CameraTest, public Test
{
public:
	Statemachine()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	int testAvailable()
	{
		/* Test operations which should fail. */
		if (camera_->configure(defconf_.get()) != -EACCES)
			return TestFail;

		if (camera_->createRequest())
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		Request request(camera_.get());
		if (camera_->queueRequest(&request) != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		if (camera_->release())
			return TestFail;

		if (camera_->stop())
			return TestFail;

		/* Test valid state transitions, end in Acquired state. */
		if (camera_->acquire())
			return TestFail;

		return TestPass;
	}

	int testAcquired()
	{
		/* Test operations which should fail. */
		if (camera_->acquire() != -EBUSY)
			return TestFail;

		if (camera_->createRequest())
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		Request request(camera_.get());
		if (camera_->queueRequest(&request) != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		if (camera_->stop())
			return TestFail;

		/* Test valid state transitions, end in Configured state. */
		if (camera_->release())
			return TestFail;

		if (camera_->acquire())
			return TestFail;

		if (camera_->configure(defconf_.get()))
			return TestFail;

		return TestPass;
	}

	int testConfigured()
	{
		/* Test operations which should fail. */
		if (camera_->acquire() != -EBUSY)
			return TestFail;

		Request request1(camera_.get());
		if (camera_->queueRequest(&request1) != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		std::unique_ptr<Request> request2 = camera_->createRequest();
		if (!request2)
			return TestFail;

		if (camera_->stop())
			return TestFail;

		/* Test valid state transitions, end in Running state. */
		if (camera_->release())
			return TestFail;

		if (camera_->acquire())
			return TestFail;

		if (camera_->configure(defconf_.get()))
			return TestFail;

		/* Use internally allocated buffers. */
		allocator_ = new FrameBufferAllocator(camera_);
		Stream *stream = *camera_->streams().begin();
		if (allocator_->allocate(stream) < 0)
			return TestFail;

		if (camera_->start())
			return TestFail;

		return TestPass;
	}

	int testRuning()
	{
		/* Test operations which should fail. */
		if (camera_->acquire() != -EBUSY)
			return TestFail;

		if (camera_->release() != -EBUSY)
			return TestFail;

		if (camera_->configure(defconf_.get()) != -EACCES)
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request)
			return TestFail;

		Stream *stream = *camera_->streams().begin();
		if (request->addBuffer(stream, allocator_->buffers(stream)[0].get()))
			return TestFail;

		if (camera_->queueRequest(request.get()))
			return TestFail;

		/* Test valid state transitions, end in Available state. */
		if (camera_->stop())
			return TestFail;

		delete allocator_;

		if (camera_->release())
			return TestFail;

		return TestPass;
	}

	int init() override
	{
		if (status_ != TestPass)
			return status_;

		defconf_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!defconf_) {
			cout << "Failed to generate default configuration" << endl;
			return TestFail;
		}

		return TestPass;
	}

	int run() override
	{
		if (testAvailable() != TestPass) {
			cout << "State machine in Available state failed" << endl;
			return TestFail;
		}

		if (testAcquired() != TestPass) {
			cout << "State machine in Acquired state failed" << endl;
			return TestFail;
		}

		if (testConfigured() != TestPass) {
			cout << "State machine in Configured state failed" << endl;
			return TestFail;
		}

		if (testRuning() != TestPass) {
			cout << "State machine in Running state failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	std::unique_ptr<CameraConfiguration> defconf_;
	FrameBufferAllocator *allocator_;
};

} /* namespace */

TEST_REGISTER(Statemachine)
