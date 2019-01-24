/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 API tests
 */

#include "v4l2_device_test.h"

class StreamOnStreamOffTest : public V4L2DeviceTest
{
protected:
	int run()
	{
		const unsigned int bufferCount = 8;

		createBuffers(bufferCount);

		int ret = dev_->exportBuffers(bufferCount, &pool_);
		if (ret)
			return TestFail;

		ret = dev_->streamOn();
		if (ret)
			return TestFail;

		ret = dev_->streamOff();
		if (ret)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(StreamOnStreamOffTest);
