/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include "v4l2_device_test.h"

class RequestBuffersTest : public V4L2DeviceTest
{
public:
	RequestBuffersTest()
		: V4L2DeviceTest("vimc", "Raw Capture 0") {}

protected:
	int run()
	{
		/*
		 * TODO:
		 *  Test invalid requests
		 *  Test different buffer allocations
		 */
		const unsigned int bufferCount = 8;

		pool_.createBuffers(bufferCount);

		int ret = capture_->exportBuffers(&pool_);
		if (ret)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(RequestBuffersTest);
