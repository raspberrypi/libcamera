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

	DeviceMatch dm("uvcvideo");
	media_ = std::move(enumerator_->search(dm));
	if (!media_)
		return TestSkip;

	media_->acquire();

	for (MediaEntity *entity : media_->entities()) {
		if (entity->flags() & MEDIA_ENT_FL_DEFAULT) {
			dev_ = new V4L2Device(entity);
			break;
		}
	}

	if (!dev_)
		return TestSkip;

	return dev_->open();
}

void V4L2DeviceTest::cleanup()
{
	media_->release();

	delete dev_;
};
