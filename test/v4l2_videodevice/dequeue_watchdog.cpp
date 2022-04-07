/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy.
 *
 * libcamera V4L2 dequeue watchdog test
 */

#include <iostream>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include <libcamera/framebuffer.h>

#include "v4l2_videodevice_test.h"

using namespace libcamera;
using namespace std::chrono_literals;

class DequeueWatchdogTest : public V4L2VideoDeviceTest
{
public:
	DequeueWatchdogTest()
		: V4L2VideoDeviceTest("vimc", "Raw Capture 0"), frames_(0), barks_(0) {}

protected:
	int run()
	{
		constexpr unsigned int bufferCount = 8;

		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		Timer timeout;

		int ret = capture_->allocateBuffers(bufferCount, &buffers_);
		if (ret < 0) {
			std::cout << "Failed to allocate buffers" << std::endl;
			return TestFail;
		}

		capture_->dequeueTimeout.connect(this, &DequeueWatchdogTest::barkCounter);
		capture_->setDequeueTimeout(5ms);

		capture_->bufferReady.connect(this, &DequeueWatchdogTest::receiveBuffer);

		for (const std::unique_ptr<FrameBuffer> &buffer : buffers_) {
			if (capture_->queueBuffer(buffer.get())) {
				std::cout << "Failed to queue buffer" << std::endl;
				return TestFail;
			}
		}

		ret = capture_->streamOn();
		if (ret < 0) {
			std::cout << "Failed to start streaming" << std::endl;
			return TestFail;
		}

		timeout.start(5s);
		while (timeout.isRunning()) {
			dispatcher->processEvents();
			if (frames_ > 5)
				break;
		}

		std::cout << "Processed " << frames_ << " frames_ and heard "
			  << barks_ << " barks_" << std::endl;

		if (!barks_) {
			std::cout << "Failed to hear any barks_." << std::endl;
			return TestFail;
		}

		capture_->streamOff();

		return TestPass;
	}

private:
	void receiveBuffer(FrameBuffer *buffer)
	{
		if (buffer->metadata().status == FrameMetadata::FrameCancelled)
			return;

		std::cout << "Buffer received" << std::endl;
		frames_++;

		/* Requeue the buffer for further use. */
		capture_->queueBuffer(buffer);
	}

	void barkCounter()
	{
		std::cout << "Watchdog is barking" << std::endl;
		barks_++;
	}

	unsigned int frames_;
	unsigned int barks_;
};

TEST_REGISTER(DequeueWatchdogTest)
