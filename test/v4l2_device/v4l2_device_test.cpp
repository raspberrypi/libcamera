/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include <iostream>
#include <sys/stat.h>

#include "v4l2_device_test.h"

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
	const std::string device("/dev/video0");

	/* Validate the device node exists. */
	if (!exists(device)) {
		std::cout << "No video device available" << std::endl;
		return TestSkip;
	}

	dev_ = new V4L2Device(device);

	return dev_->open();
}

void V4L2DeviceTest::cleanup()
{
	delete dev_;
};
