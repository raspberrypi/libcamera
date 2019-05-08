/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include <libcamera/buffer.h>
#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>
#include <libcamera/timer.h>

#include <iostream>

#include "v4l2_device_test.h"

class CaptureAsyncTest : public V4L2DeviceTest
{
public:
	CaptureAsyncTest()
		: V4L2DeviceTest("vimc", "Raw Capture 0"), frames(0) {}

	void receiveBuffer(Buffer *buffer)
	{
		std::cout << "Received buffer " << buffer->index() << std::endl;
		frames++;

		/* Requeue the buffer for further use. */
		capture_->queueBuffer(buffer);
	}

protected:
	int run()
	{
		const unsigned int bufferCount = 8;

		EventDispatcher *dispatcher = CameraManager::instance()->eventDispatcher();
		Timer timeout;
		int ret;

		pool_.createBuffers(bufferCount);

		ret = capture_->exportBuffers(&pool_);
		if (ret)
			return TestFail;

		capture_->bufferReady.connect(this, &CaptureAsyncTest::receiveBuffer);

		/* Queue all the buffers to the device. */
		for (Buffer &b : pool_.buffers()) {
			if (capture_->queueBuffer(&b))
				return TestFail;
		}

		ret = capture_->streamOn();
		if (ret)
			return TestFail;

		timeout.start(10000);
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

TEST_REGISTER(CaptureAsyncTest);
