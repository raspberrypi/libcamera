/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 *
 * Validate the function of exporting buffers from a V4L2Device and
 * the ability to import them to another V4L2Device instance.
 * Ensure that the Buffers can successfully be queued and dequeued
 * between both devices.
 */

#include <iostream>

#include <libcamera/buffer.h>
#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>
#include <libcamera/timer.h>

#include "v4l2_device_test.h"

class BufferSharingTest : public V4L2DeviceTest
{
public:
	BufferSharingTest()
		: V4L2DeviceTest("vivid", "vivid-000-vid-cap"),
		  output_(nullptr), framesCaptured_(0), framesOutput_(0) {}

protected:
	int init()
	{
		int ret = V4L2DeviceTest::init();
		if (ret)
			return ret;

		/* media_ already represents VIVID */
		MediaEntity *entity = media_->getEntityByName("vivid-000-vid-out");
		if (!entity)
			return TestSkip;

		output_ = new V4L2Device(entity);
		if (!output_) {
			std::cout << "Failed to create output device" << std::endl;
			return TestFail;
		}

		ret = output_->open();
		if (ret) {
			std::cout << "Failed to open output device" << std::endl;
			return TestFail;
		}

		V4L2DeviceFormat format = {};

		ret = capture_->getFormat(&format);
		if (ret) {
			std::cout << "Failed to get capture format" << std::endl;
			return TestFail;
		}

		ret = output_->setFormat(&format);
		if (ret) {
			std::cout << "Failed to set output format" << std::endl;
			return TestFail;
		}

		pool_.createBuffers(bufferCount);

		ret = capture_->exportBuffers(&pool_);
		if (ret) {
			std::cout << "Failed to export buffers" << std::endl;
			return TestFail;
		}

		ret = output_->importBuffers(&pool_);
		if (ret) {
			std::cout << "Failed to import buffers" << std::endl;
			return TestFail;
		}

		return 0;
	}

	void captureBufferReady(Buffer *buffer)
	{
		std::cout << "Received capture buffer: " << buffer->index()
			  << " sequence " << buffer->sequence() << std::endl;

		output_->queueBuffer(buffer);
		framesCaptured_++;
	}

	void outputBufferReady(Buffer *buffer)
	{
		std::cout << "Received output buffer: " << buffer->index()
			  << " sequence " << buffer->sequence() << std::endl;

		capture_->queueBuffer(buffer);
		framesOutput_++;
	}

	int run()
	{
		EventDispatcher *dispatcher = CameraManager::instance()->eventDispatcher();
		Timer timeout;
		int ret;

		capture_->bufferReady.connect(this, &BufferSharingTest::captureBufferReady);
		output_->bufferReady.connect(this, &BufferSharingTest::outputBufferReady);

		/* Queue all the buffers to the capture device. */
		for (Buffer &buffer : pool_.buffers()) {
			if (capture_->queueBuffer(&buffer))
				return TestFail;
		}

		ret = capture_->streamOn();
		if (ret) {
			std::cout << "Failed to start streaming on the capture device" << std::endl;
			return TestFail;
		}

		ret = output_->streamOn();
		if (ret) {
			std::cout << "Failed to start streaming on the output device" << std::endl;
			return TestFail;
		}

		timeout.start(10000);
		while (timeout.isRunning()) {
			dispatcher->processEvents();
			if (framesCaptured_ > 30 && framesOutput_ > 30)
				break;
		}

		if ((framesCaptured_ < 1) || (framesOutput_ < 1)) {
			std::cout << "Failed to process any frames within timeout." << std::endl;
			return TestFail;
		}

		if ((framesCaptured_ < 30) || (framesOutput_ < 30)) {
			std::cout << "Failed to process 30 frames within timeout." << std::endl;
			return TestFail;
		}

		ret = capture_->streamOff();
		if (ret) {
			std::cout << "Failed to stop streaming on the capture device" << std::endl;
			return TestFail;
		}

		ret = output_->streamOff();
		if (ret) {
			std::cout << "Failed to stop streaming on the output device" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		std::cout
			<< "Captured " << framesCaptured_ << " frames and "
			<< "output " << framesOutput_ << " frames"
			<< std::endl;

		output_->streamOff();
		output_->releaseBuffers();
		output_->close();

		delete output_;

		V4L2DeviceTest::cleanup();
	}

private:
	const unsigned int bufferCount = 4;

	V4L2Device *output_;

	unsigned int framesCaptured_;
	unsigned int framesOutput_;
};

TEST_REGISTER(BufferSharingTest);
