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

#include "device_enumerator.h"
#include "media_device.h"
#include "v4l2_videodevice.h"

#include "camera_test.h"
#include "test.h"

using namespace libcamera;

namespace {

/* A provider of external buffers, suitable for import by a Camera. */
class BufferSource
{
public:
	BufferSource()
		: video_(nullptr)
	{
	}

	~BufferSource()
	{
		if (video_) {
			video_->releaseBuffers();
			video_->close();
		}

		delete video_;
		video_ = nullptr;

		if (media_)
			media_->release();
	}

	int allocate(const StreamConfiguration &config)
	{
		/* Locate and open the video device. */
		std::string videoDeviceName = "vivid-000-vid-out";

		std::unique_ptr<DeviceEnumerator> enumerator =
			DeviceEnumerator::create();
		if (!enumerator) {
			std::cout << "Failed to create device enumerator" << std::endl;
			return TestFail;
		}

		if (enumerator->enumerate()) {
			std::cout << "Failed to enumerate media devices" << std::endl;
			return TestFail;
		}

		DeviceMatch dm("vivid");
		dm.add(videoDeviceName);

		media_ = enumerator->search(dm);
		if (!media_) {
			std::cout << "No vivid output device available" << std::endl;
			return TestSkip;
		}

		video_ = V4L2VideoDevice::fromEntityName(media_.get(), videoDeviceName);
		if (!video_) {
			std::cout << "Failed to get video device from entity "
				  << videoDeviceName << std::endl;
			return TestFail;
		}

		if (video_->open()) {
			std::cout << "Unable to open " << videoDeviceName << std::endl;
			return TestFail;
		}

		/* Configure the format. */
		V4L2DeviceFormat format;
		if (video_->getFormat(&format)) {
			std::cout << "Failed to get format on output device" << std::endl;
			return TestFail;
		}

		format.size = config.size;
		format.fourcc = V4L2VideoDevice::toV4L2Fourcc(config.pixelFormat, false);
		if (video_->setFormat(&format)) {
			std::cout << "Failed to set format on output device" << std::endl;
			return TestFail;
		}

		if (video_->exportBuffers(config.bufferCount, &buffers_) < 0) {
			std::cout << "Failed to export buffers" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	const std::vector<std::unique_ptr<FrameBuffer>> &buffers()
	{
		return buffers_;
	}

private:
	std::shared_ptr<MediaDevice> media_;
	V4L2VideoDevice *video_;
	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
};

class BufferImportTest : public CameraTest, public Test
{
public:
	BufferImportTest()
		: CameraTest("VIMC Sensor B")
	{
	}

protected:
	void bufferComplete(Request *request, FrameBuffer *buffer)
	{
		if (buffer->metadata().status != FrameMetadata::FrameSuccess)
			return;

		completeBuffersCount_++;
	}

	void requestComplete(Request *request)
	{
		if (request->status() != Request::RequestComplete)
			return;

		const std::map<Stream *, FrameBuffer *> &buffers = request->buffers();

		completeRequestsCount_++;

		/* Create a new request. */
		Stream *stream = buffers.begin()->first;
		FrameBuffer *buffer = buffers.begin()->second;

		request = camera_->createRequest();
		request->addBuffer(stream, buffer);
		camera_->queueRequest(request);
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

		std::vector<Request *> requests;
		for (const std::unique_ptr<FrameBuffer> &buffer : source.buffers()) {
			Request *request = camera_->createRequest();
			if (!request) {
				std::cout << "Failed to create request" << std::endl;
				return TestFail;
			}

			if (request->addBuffer(stream, buffer.get())) {
				std::cout << "Failed to associating buffer with request" << std::endl;
				return TestFail;
			}

			requests.push_back(request);
		}

		completeRequestsCount_ = 0;
		completeBuffersCount_ = 0;

		camera_->bufferCompleted.connect(this, &BufferImportTest::bufferComplete);
		camera_->requestCompleted.connect(this, &BufferImportTest::requestComplete);

		if (camera_->start()) {
			std::cout << "Failed to start camera" << std::endl;
			return TestFail;
		}

		for (Request *request : requests) {
			if (camera_->queueRequest(request)) {
				std::cout << "Failed to queue request" << std::endl;
				return TestFail;
			}
		}

		EventDispatcher *dispatcher = cm_->eventDispatcher();

		Timer timer;
		timer.start(1000);
		while (timer.isRunning())
			dispatcher->processEvents();

		if (completeRequestsCount_ <= cfg.bufferCount * 2) {
			std::cout << "Failed to capture enough frames (got "
				  << completeRequestsCount_ << " expected at least "
				  << cfg.bufferCount * 2 << ")" << std::endl;
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
	unsigned int completeBuffersCount_;
	unsigned int completeRequestsCount_;
	std::unique_ptr<CameraConfiguration> config_;
};

} /* namespace */

TEST_REGISTER(BufferImportTest);
