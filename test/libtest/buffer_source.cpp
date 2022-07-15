/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * A provider of external buffers, suitable for use in tests.
 */

#include "buffer_source.h"

#include <iostream>
#include <memory>

#include "libcamera/internal/device_enumerator.h"

#include "test.h"

using namespace libcamera;

BufferSource::BufferSource()
{
}

BufferSource::~BufferSource()
{
	if (media_)
		media_->release();
}

int BufferSource::allocate(const StreamConfiguration &config)
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

	std::unique_ptr<V4L2VideoDevice> video = V4L2VideoDevice::fromEntityName(media_.get(), videoDeviceName);
	if (!video) {
		std::cout << "Failed to get video device from entity "
			  << videoDeviceName << std::endl;
		return TestFail;
	}

	if (video->open()) {
		std::cout << "Unable to open " << videoDeviceName << std::endl;
		return TestFail;
	}

	/* Configure the format. */
	V4L2DeviceFormat format;
	if (video->getFormat(&format)) {
		std::cout << "Failed to get format on output device" << std::endl;
		return TestFail;
	}

	format.size = config.size;
	format.fourcc = video->toV4L2PixelFormat(config.pixelFormat);
	if (video->setFormat(&format)) {
		std::cout << "Failed to set format on output device" << std::endl;
		return TestFail;
	}

	if (video->allocateBuffers(config.bufferCount, &buffers_) < 0) {
		std::cout << "Failed to allocate buffers" << std::endl;
		return TestFail;
	}

	video->close();

	return TestPass;
}

const std::vector<std::unique_ptr<FrameBuffer>> &BufferSource::buffers()
{
	return buffers_;
}
