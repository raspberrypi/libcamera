/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 *  media_device_acquire.cpp- Test acquire/release of a MediaDevice
 */

#include "media_device_test.h"

using namespace libcamera;

class MediaDeviceAcquire : public MediaDeviceTest
{
	int run()
	{
		if (!media_->acquire())
			return TestFail;

		if (media_->acquire())
			return TestFail;

		media_->release();

		if (!media_->acquire())
			return TestFail;

		media_->release();

		return TestPass;
	}
};

TEST_REGISTER(MediaDeviceAcquire)
