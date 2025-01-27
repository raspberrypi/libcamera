/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include <iostream>

#include <linux/media-bus-format.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"

#include "v4l2_videodevice_test.h"

using namespace std;
using namespace libcamera;

int V4L2VideoDeviceTest::init()
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

	DeviceMatch dm(driver_);
	dm.add(entity_);

	media_ = enumerator_->search(dm);
	if (!media_)
		return TestSkip;

	MediaEntity *entity = media_->getEntityByName(entity_);
	if (!entity)
		return TestSkip;

	capture_ = new V4L2VideoDevice(entity);
	if (!capture_)
		return TestFail;

	if (!media_->acquire())
		return TestFail;

	int ret = media_->disableLinks();
	media_->release();
	if (ret)
		return TestFail;

	if (capture_->open())
		return TestFail;

	V4L2DeviceFormat format = {};
	if (capture_->getFormat(&format))
		return TestFail;

	format.size.width = 640;
	format.size.height = 480;

	if (driver_ == "vimc") {
		sensor_ = CameraSensorFactoryBase::create(media_->getEntityByName("Sensor A"));
		if (!sensor_)
			return TestFail;

		debayer_ = new V4L2Subdevice(media_->getEntityByName("Debayer A"));
		if (debayer_->open())
			return TestFail;

		format.fourcc = V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8);

		V4L2SubdeviceFormat subformat = {};
		subformat.code = MEDIA_BUS_FMT_SBGGR8_1X8;
		subformat.size = format.size;

		if (sensor_->setFormat(&subformat))
			return TestFail;

		if (debayer_->setFormat(0, &subformat))
			return TestFail;
	}

	if (capture_->setFormat(&format))
		return TestFail;

	return TestPass;
}

void V4L2VideoDeviceTest::cleanup()
{
	capture_->streamOff();
	capture_->releaseBuffers();
	capture_->close();

	delete debayer_;
	delete capture_;
}
