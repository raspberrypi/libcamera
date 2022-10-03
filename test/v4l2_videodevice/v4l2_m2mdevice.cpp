/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 M2M video device tests
 */

#include <iostream>

#include <libcamera/framebuffer.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

class V4L2M2MDeviceTest : public Test
{
public:
	V4L2M2MDeviceTest()
		: vim2m_(nullptr), outputFrames_(0), captureFrames_(0)
	{
	}

	void outputBufferComplete(FrameBuffer *buffer)
	{
		cout << "Received output buffer" << endl;

		outputFrames_++;

		/* Requeue the buffer for further use. */
		vim2m_->output()->queueBuffer(buffer);
	}

	void receiveCaptureBuffer(FrameBuffer *buffer)
	{
		cout << "Received capture buffer" << endl;

		captureFrames_++;

		/* Requeue the buffer for further use. */
		vim2m_->capture()->queueBuffer(buffer);
	}

protected:
	int init()
	{
		enumerator_ = DeviceEnumerator::create();
		if (!enumerator_) {
			cerr << "Failed to create device enumerator" << endl;
			return TestFail;
		}

		if (enumerator_->enumerate()) {
			cerr << "Failed to enumerate media devices" << endl;
			return TestFail;
		}

		DeviceMatch dm("vim2m");
		dm.add("vim2m-source");
		dm.add("vim2m-sink");

		media_ = enumerator_->search(dm);
		if (!media_) {
			cerr << "No vim2m device found" << endl;
			return TestSkip;
		}

		return TestPass;
	}

	int run()
	{
		constexpr unsigned int bufferCount = 4;

		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		int ret;

		MediaEntity *entity = media_->getEntityByName("vim2m-source");
		vim2m_ = new V4L2M2MDevice(entity->deviceNode());
		if (vim2m_->open()) {
			cerr << "Failed to open VIM2M device" << endl;
			return TestFail;
		}

		V4L2VideoDevice *capture = vim2m_->capture();
		V4L2VideoDevice *output = vim2m_->output();

		if (capture->controls().empty() || output->controls().empty()) {
			cerr << "VIM2M device has no control" << endl;
			return TestFail;
		}

		V4L2DeviceFormat format = {};
		if (capture->getFormat(&format)) {
			cerr << "Failed to get capture format" << endl;
			return TestFail;
		}

		format.size.width = 640;
		format.size.height = 480;

		if (capture->setFormat(&format)) {
			cerr << "Failed to set capture format" << endl;
			return TestFail;
		}

		if (output->setFormat(&format)) {
			cerr << "Failed to set output format" << endl;
			return TestFail;
		}

		ret = capture->allocateBuffers(bufferCount, &captureBuffers_);
		if (ret < 0) {
			cerr << "Failed to allocate Capture Buffers" << endl;
			return TestFail;
		}

		ret = output->allocateBuffers(bufferCount, &outputBuffers_);
		if (ret < 0) {
			cerr << "Failed to allocate Output Buffers" << endl;
			return TestFail;
		}

		capture->bufferReady.connect(this, &V4L2M2MDeviceTest::receiveCaptureBuffer);
		output->bufferReady.connect(this, &V4L2M2MDeviceTest::outputBufferComplete);

		for (const std::unique_ptr<FrameBuffer> &buffer : captureBuffers_) {
			if (capture->queueBuffer(buffer.get())) {
				std::cout << "Failed to queue capture buffer" << std::endl;
				return TestFail;
			}
		}

		for (const std::unique_ptr<FrameBuffer> &buffer : outputBuffers_) {
			if (output->queueBuffer(buffer.get())) {
				std::cout << "Failed to queue output buffer" << std::endl;
				return TestFail;
			}
		}

		ret = capture->streamOn();
		if (ret) {
			cerr << "Failed to streamOn capture" << endl;
			return TestFail;
		}

		ret = output->streamOn();
		if (ret) {
			cerr << "Failed to streamOn output" << endl;
			return TestFail;
		}

		Timer timeout;
		timeout.start(5000ms);
		while (timeout.isRunning()) {
			dispatcher->processEvents();
			if (captureFrames_ > 30)
				break;
		}

		cerr << "Output " << outputFrames_ << " frames" << std::endl;
		cerr << "Captured " << captureFrames_ << " frames" << std::endl;

		if (captureFrames_ < 30) {
			cerr << "Failed to capture 30 frames within timeout." << std::endl;
			return TestFail;
		}

		ret = capture->streamOff();
		if (ret) {
			cerr << "Failed to StreamOff the capture device." << std::endl;
			return TestFail;
		}

		ret = output->streamOff();
		if (ret) {
			cerr << "Failed to StreamOff the output device." << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		delete vim2m_;
	}

private:
	std::unique_ptr<DeviceEnumerator> enumerator_;
	std::shared_ptr<MediaDevice> media_;
	V4L2M2MDevice *vim2m_;

	std::vector<std::unique_ptr<FrameBuffer>> captureBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> outputBuffers_;

	unsigned int outputFrames_;
	unsigned int captureFrames_;
};

TEST_REGISTER(V4L2M2MDeviceTest)
