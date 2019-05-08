/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 device format handling test
 */

#include <climits>
#include <iostream>

#include "v4l2_device.h"

#include "v4l2_device_test.h"

using namespace std;
using namespace libcamera;

class Format : public V4L2DeviceTest
{
public:
	Format()
		: V4L2DeviceTest("vimc", "Raw Capture 0") {}
protected:
	int run()
	{
		V4L2DeviceFormat format = {};

		int ret = capture_->getFormat(&format);
		if (ret) {
			cerr << "Failed to get format" << endl;
			return TestFail;
		}

		format.size = { UINT_MAX, UINT_MAX };
		ret = capture_->setFormat(&format);
		if (ret) {
			cerr << "Failed to set format: image resolution is invalid: "
			     << "(UINT_MAX x UINT_MAX) but setFormat() should not fail."
			     << endl;
			return TestFail;
		}

		if (format.size.width == UINT_MAX ||
		    format.size.height == UINT_MAX) {
			cerr << "Failed to update image format = (UINT_MAX x UINT_MAX)"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(Format);
