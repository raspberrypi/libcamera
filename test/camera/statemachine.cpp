/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include "camera_test.h"

using namespace std;

namespace {

class Statemachine : public CameraTest
{
protected:
	int testAvailable()
	{
		/* Test operations which should fail. */
		if (camera_->configure(defconf_.get()) != -EACCES)
			return TestFail;

		if (camera_->allocateBuffers() != -EACCES)
			return TestFail;

		if (camera_->freeBuffers() != -EACCES)
			return TestFail;

		if (camera_->createRequest())
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		Request request(camera_.get());
		if (camera_->queueRequest(&request) != -EACCES)
			return TestFail;

		if (camera_->stop() != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		if (camera_->release())
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

		if (camera_->allocateBuffers() != -EACCES)
			return TestFail;

		if (camera_->freeBuffers() != -EACCES)
			return TestFail;

		if (camera_->createRequest())
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		Request request(camera_.get());
		if (camera_->queueRequest(&request) != -EACCES)
			return TestFail;

		if (camera_->stop() != -EACCES)
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

		if (camera_->freeBuffers() != -EACCES)
			return TestFail;

		if (camera_->createRequest())
			return TestFail;

		Request request(camera_.get());
		if (camera_->queueRequest(&request) != -EACCES)
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		if (camera_->stop() != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		if (camera_->configure(defconf_.get()))
			return TestFail;

		/* Test valid state transitions, end in Prepared state. */
		if (camera_->release())
			return TestFail;

		if (camera_->acquire())
			return TestFail;

		if (camera_->configure(defconf_.get()))
			return TestFail;

		if (camera_->allocateBuffers())
			return TestFail;

		return TestPass;
	}

	int testPrepared()
	{
		/* Test operations which should fail. */
		if (camera_->acquire() != -EBUSY)
			return TestFail;

		if (camera_->release() != -EBUSY)
			return TestFail;

		if (camera_->configure(defconf_.get()) != -EACCES)
			return TestFail;

		if (camera_->allocateBuffers() != -EACCES)
			return TestFail;

		Request request1(camera_.get());
		if (camera_->queueRequest(&request1) != -EACCES)
			return TestFail;

		if (camera_->stop() != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		Request *request2 = camera_->createRequest();
		if (!request2)
			return TestFail;

		/* Never handed to hardware so need to manually delete it. */
		delete request2;

		/* Test valid state transitions, end in Running state. */
		if (camera_->freeBuffers())
			return TestFail;

		if (camera_->release())
			return TestFail;

		if (camera_->acquire())
			return TestFail;

		if (camera_->configure(defconf_.get()))
			return TestFail;

		if (camera_->allocateBuffers())
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

		if (camera_->allocateBuffers() != -EACCES)
			return TestFail;

		if (camera_->freeBuffers() != -EACCES)
			return TestFail;

		if (camera_->start() != -EACCES)
			return TestFail;

		/* Test operations which should pass. */
		Request *request = camera_->createRequest();
		if (!request)
			return TestFail;

		Stream *stream = *camera_->streams().begin();
		BufferPool &pool = stream->bufferPool();
		Buffer &buffer = pool.buffers().front();
		std::map<Stream *, Buffer *> map = { { stream, &buffer } };
		if (request->setBuffers(map))
			return TestFail;

		if (camera_->queueRequest(request))
			return TestFail;

		/* Test valid state transitions, end in Available state. */
		if (camera_->stop())
			return TestFail;

		if (camera_->freeBuffers())
			return TestFail;

		if (camera_->release())
			return TestFail;

		return TestPass;
	}

	int init() override
	{
		CameraTest::init();

		defconf_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!defconf_) {
			cout << "Failed to generate default configuration" << endl;
			CameraTest::cleanup();
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

		if (testPrepared() != TestPass) {
			cout << "State machine in Prepared state failed" << endl;
			return TestFail;
		}

		if (testRuning() != TestPass) {
			cout << "State machine in Running state failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	std::unique_ptr<CameraConfiguration> defconf_;
};

} /* namespace */

TEST_REGISTER(Statemachine);
