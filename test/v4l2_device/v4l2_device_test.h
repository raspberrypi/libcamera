/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vl42device_test.h - libcamera v4l2device test base class
 */
#ifndef __LIBCAMERA_V4L2_DEVICE_TEST_H_
#define __LIBCAMERA_V4L2_DEVICE_TEST_H_

#include "test.h"
#include "v4l2_device.h"

using namespace libcamera;

class V4L2DeviceTest : public Test
{
public:
	V4L2DeviceTest() : dev_(nullptr) { };

protected:
	int init();
	void cleanup();

	V4L2Device *dev_;
};

#endif /* __LIBCAMERA_V4L2_DEVICE_TEST_H_ */
