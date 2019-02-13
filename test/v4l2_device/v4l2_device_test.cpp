/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include <iostream>
#include <sys/stat.h>

#include "v4l2_device_test.h"

#include "device_enumerator.h"
#include "media_device.h"

using namespace std;
using namespace libcamera;

bool exists(const std::string &path)
{
	struct stat sb;

	if (stat(path.c_str(), &sb) == 0)
		return true;

	return false;
}

int V4L2DeviceTest::init()
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

	DeviceMatch dm("vivid");
	dm.add("vivid-000-vid-cap");

	media_ = enumerator_->search(dm);
	if (!media_)
		return TestSkip;

	media_->acquire();

	MediaEntity *entity = media_->getEntityByName("vivid-000-vid-cap");
	if (!entity)
		return TestSkip;

	capture_ = new V4L2Device(entity);
	if (!capture_)
		return TestFail;

	return capture_->open();
}

void V4L2DeviceTest::cleanup()
{
	media_->release();

	capture_->streamOff();
	capture_->releaseBuffers();
	capture_->close();

	delete capture_;
};
