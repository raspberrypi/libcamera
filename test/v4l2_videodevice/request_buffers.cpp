/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include "v4l2_videodevice_test.h"

class RequestBuffersTest : public V4L2VideoDeviceTest
{
public:
	RequestBuffersTest()
		: V4L2VideoDeviceTest("vimc", "Raw Capture 0") {}

protected:
	int run()
	{
		const unsigned int bufferCount = 8;

		int ret = capture_->allocateBuffers(bufferCount, &buffers_);
		if (ret != bufferCount)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(RequestBuffersTest)
