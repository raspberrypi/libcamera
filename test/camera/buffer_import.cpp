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
#include <random>
#include <vector>

#include "device_enumerator.h"
#include "media_device.h"
#include "v4l2_videodevice.h"

#include "camera_test.h"

using namespace libcamera;

/* Keep SINK_BUFFER_COUNT > CAMERA_BUFFER_COUNT + 1 */
static constexpr unsigned int SINK_BUFFER_COUNT = 8;
static constexpr unsigned int CAMERA_BUFFER_COUNT = 4;

class FrameSink
{
public:
	FrameSink()
		: video_(nullptr)
	{
	}

	int init()
	{
		int ret;

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
			std::cout << "Unable to open " << videoDeviceName << std::endl;
			return TestFail;
		}

		if (video_->open())
			return TestFail;

		/* Configure the format. */
		ret = video_->getFormat(&format_);
		if (ret) {
			std::cout << "Failed to get format on output device" << std::endl;
			return ret;
		}

		format_.size.width = 1920;
		format_.size.height = 1080;
		format_.fourcc = V4L2_PIX_FMT_RGB24;
		format_.planesCount = 1;
		format_.planes[0].size = 1920 * 1080 * 3;
		format_.planes[0].bpl = 1920 * 3;

		if (video_->setFormat(&format_)) {
			cleanup();
			return TestFail;
		}

		/* Export the buffers to a pool. */
		pool_.createBuffers(SINK_BUFFER_COUNT);
		ret = video_->exportBuffers(&pool_);
		if (ret) {
			std::cout << "Failed to export buffers" << std::endl;
			cleanup();
			return TestFail;
		}

		/* Only use the first CAMERA_BUFFER_COUNT buffers to start with. */
		availableBuffers_.resize(CAMERA_BUFFER_COUNT);
		std::iota(availableBuffers_.begin(), availableBuffers_.end(), 0);

		/* Connect the buffer ready signal. */
		video_->bufferReady.connect(this, &FrameSink::bufferComplete);

		return TestPass;
	}

	void cleanup()
	{
		if (video_) {
			video_->streamOff();
			video_->releaseBuffers();
			video_->close();

			delete video_;
			video_ = nullptr;
		}

		if (media_)
			media_->release();
	}

	int start()
	{
		requestsCount_ = 0;
		done_ = false;

		int ret = video_->streamOn();
		if (ret < 0)
			return ret;

		/* Queue all the initial requests. */
		for (unsigned int index = 0; index < CAMERA_BUFFER_COUNT; ++index)
			queueRequest(index);

		return 0;
	}

	int stop()
	{
		return video_->streamOff();
	}

	void requestComplete(uint64_t cookie, const Buffer *metadata)
	{
		unsigned int index = cookie;

		Buffer *buffer = new Buffer(index, metadata);
		int ret = video_->queueBuffer(buffer);
		if (ret < 0)
			std::cout << "Failed to queue buffer to sink" << std::endl;
	}

	bool done() const { return done_; }

	PixelFormat format() const
	{
		return video_->toPixelFormat(format_.fourcc);
	}

	const Size &size() const
	{
		return format_.size;
	}

	Signal<uint64_t, int> requestReady;

private:
	void queueRequest(unsigned int index)
	{
		auto it = std::find(availableBuffers_.begin(),
				    availableBuffers_.end(), index);
		availableBuffers_.erase(it);

		uint64_t cookie = index;
		BufferMemory &mem = pool_.buffers()[index];
		int dmabuf = mem.planes()[0].dmabuf();

		requestReady.emit(cookie, dmabuf);

		requestsCount_++;
	}

	void bufferComplete(Buffer *buffer)
	{
		availableBuffers_.push_back(buffer->index());

		/*
		 * Pick the buffer for the next request among the available
		 * buffers.
		 *
		 * For the first 20 frames, select the buffer that has just
		 * completed to keep the mapping of dmabuf fds to buffers
		 * unchanged in the camera.
		 *
		 * For the next 20 frames, cycle randomly over the available
		 * buffers. The mapping should still be kept unchanged, as the
		 * camera should map using the cached fds.
		 *
		 * For the last 20 frames, cycles through all buffers, which
		 * should trash the mappings.
		 */
		unsigned int index = buffer->index();
		delete buffer;

		std::cout << "Completed buffer, request=" << requestsCount_
			  << ", available buffers=" << availableBuffers_.size()
			  << std::endl;

		if (requestsCount_ >= 60) {
			if (availableBuffers_.size() == SINK_BUFFER_COUNT)
				done_ = true;
			return;
		}

		if (requestsCount_ == 40) {
			/* Add the remaining of the buffers. */
			for (unsigned int i = CAMERA_BUFFER_COUNT;
			     i < SINK_BUFFER_COUNT; ++i)
				availableBuffers_.push_back(i);
		}

		if (requestsCount_ >= 20) {
			/*
			 * Wait until we have enough buffers to make this
			 * meaningful. Preferably half of the camera buffers,
			 * but no less than 2 in any case.
			 */
			const unsigned int min_pool_size =
				std::min(CAMERA_BUFFER_COUNT / 2, 2U);
			if (availableBuffers_.size() < min_pool_size)
				return;

			/* Pick a buffer at random. */
			unsigned int pos = random_() % availableBuffers_.size();
			index = availableBuffers_[pos];
		}

		queueRequest(index);
	}

	std::shared_ptr<MediaDevice> media_;
	V4L2VideoDevice *video_;
	BufferPool pool_;
	V4L2DeviceFormat format_;

	unsigned int requestsCount_;
	std::vector<int> availableBuffers_;
	std::random_device random_;

	bool done_;
};

class BufferImportTest : public CameraTest
{
public:
	BufferImportTest()
		: CameraTest()
	{
	}

	void queueRequest(uint64_t cookie, int dmabuf)
	{
		Request *request = camera_->createRequest(cookie);

		std::unique_ptr<Buffer> buffer = stream_->createBuffer({ dmabuf, -1, -1 });
		request->addBuffer(move(buffer));
		camera_->queueRequest(request);
	}

protected:
	void bufferComplete(Request *request, Buffer *buffer)
	{
		if (buffer->status() != Buffer::BufferSuccess)
			return;

		unsigned int index = buffer->index();
		int dmabuf = buffer->dmabufs()[0];

		/* Record dmabuf to index remappings. */
		bool remapped = false;
		if (bufferMappings_.find(index) != bufferMappings_.end()) {
			if (bufferMappings_[index] != dmabuf)
				remapped = true;
		}

		std::cout << "Completed request " << framesCaptured_
			  << ": dmabuf fd " << dmabuf
			  << " -> index " << index
			  << " (" << (remapped ? 'R' : '-') << ")"
			  << std::endl;

		if (remapped)
			bufferRemappings_.push_back(framesCaptured_);

		bufferMappings_[index] = dmabuf;
		framesCaptured_++;

		sink_.requestComplete(request->cookie(), buffer);

		if (framesCaptured_ == 60)
			sink_.stop();
	}

	int initCamera()
	{
		if (camera_->acquire()) {
			std::cout << "Failed to acquire the camera" << std::endl;
			return TestFail;
		}

		/*
		 * Configure the Stream to work with externally allocated
		 * buffers by setting the memoryType to ExternalMemory.
		 */
		std::unique_ptr<CameraConfiguration> config;
		config = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!config || config->size() != 1) {
			std::cout << "Failed to generate configuration" << std::endl;
			return TestFail;
		}

		StreamConfiguration &cfg = config->at(0);
		cfg.size = sink_.size();
		cfg.pixelFormat = sink_.format();
		cfg.bufferCount = CAMERA_BUFFER_COUNT;
		cfg.memoryType = ExternalMemory;

		if (camera_->configure(config.get())) {
			std::cout << "Failed to set configuration" << std::endl;
			return TestFail;
		}

		stream_ = cfg.stream();

		/* Allocate buffers. */
		if (camera_->allocateBuffers()) {
			std::cout << "Failed to allocate buffers" << std::endl;
			return TestFail;
		}

		/* Connect the buffer completed signal. */
		camera_->bufferCompleted.connect(this, &BufferImportTest::bufferComplete);

		return TestPass;
	}

	int init()
	{
		int ret = CameraTest::init();
		if (ret)
			return ret;

		ret = sink_.init();
		if (ret != TestPass) {
			cleanup();
			return ret;
		}

		ret = initCamera();
		if (ret != TestPass) {
			cleanup();
			return ret;
		}

		sink_.requestReady.connect(this, &BufferImportTest::queueRequest);
		return TestPass;
	}

	int run()
	{
		int ret;

		framesCaptured_ = 0;

		if (camera_->start()) {
			std::cout << "Failed to start camera" << std::endl;
			return TestFail;
		}

		ret = sink_.start();
		if (ret < 0) {
			std::cout << "Failed to start sink" << std::endl;
			return TestFail;
		}

		EventDispatcher *dispatcher = cm_->eventDispatcher();

		Timer timer;
		timer.start(5000);
		while (timer.isRunning() && !sink_.done())
			dispatcher->processEvents();

		std::cout << framesCaptured_ << " frames captured, "
			  << bufferRemappings_.size() << " buffers remapped"
			  << std::endl;

		if (framesCaptured_ < 60) {
			std::cout << "Too few frames captured" << std::endl;
			return TestFail;
		}

		if (bufferRemappings_.empty()) {
			std::cout << "No buffer remappings" << std::endl;
			return TestFail;
		}

		if (bufferRemappings_[0] < 40) {
			std::cout << "Early buffer remapping" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		sink_.cleanup();

		camera_->stop();
		camera_->freeBuffers();

		CameraTest::cleanup();
	}

private:
	Stream *stream_;

	std::map<unsigned int, int> bufferMappings_;
	std::vector<unsigned int> bufferRemappings_;
	unsigned int framesCaptured_;

	FrameSink sink_;
};

TEST_REGISTER(BufferImportTest);
