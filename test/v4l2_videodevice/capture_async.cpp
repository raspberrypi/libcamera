/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include <iostream>

#include <libcamera/framebuffer.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "v4l2_videodevice_test.h"

using namespace libcamera;
using namespace std::chrono_literals;

class CaptureAsyncTest : public V4L2VideoDeviceTest
{
public:
	CaptureAsyncTest()
		: V4L2VideoDeviceTest("vimc", "Raw Capture 0"), frames(0) {}

	void receiveBuffer(FrameBuffer *buffer)
	{
		std::cout << "Buffer received" << std::endl;
		frames++;

		/* Requeue the buffer for further use. */
		capture_->queueBuffer(buffer);
	}

protected:
	int run()
	{
		const unsigned int bufferCount = 8;

		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		Timer timeout;
		int ret;

		ret = capture_->allocateBuffers(bufferCount, &buffers_);
		if (ret < 0) {
			std::cout << "Failed to allocate buffers" << std::endl;
			return TestFail;
		}

		capture_->bufferReady.connect(this, &CaptureAsyncTest::receiveBuffer);

		for (const std::unique_ptr<FrameBuffer> &buffer : buffers_) {
			if (capture_->queueBuffer(buffer.get())) {
				std::cout << "Failed to queue buffer" << std::endl;
				return TestFail;
			}
		}

		ret = capture_->streamOn();
		if (ret)
			return TestFail;

		timeout.start(10000ms);
		while (timeout.isRunning()) {
			dispatcher->processEvents();
			if (frames > 30)
				break;
		}

		if (frames < 1) {
			std::cout << "Failed to capture any frames within timeout." << std::endl;
			return TestFail;
		}

		if (frames < 30) {
			std::cout << "Failed to capture 30 frames within timeout." << std::endl;
			return TestFail;
		}

		std::cout << "Processed " << frames << " frames" << std::endl;

		ret = capture_->streamOff();
		if (ret)
			return TestFail;

		return TestPass;
	}

private:
	unsigned int frames;
};

TEST_REGISTER(CaptureAsyncTest)
