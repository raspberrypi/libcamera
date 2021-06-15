/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 device format handling test
 */

#include <iostream>
#include <limits.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/v4l2_videodevice.h"

#include "v4l2_videodevice_test.h"

using namespace std;
using namespace libcamera;

class Format : public V4L2VideoDeviceTest
{
public:
	Format()
		: V4L2VideoDeviceTest("vimc", "Raw Capture 0") {}
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

		std::vector<std::pair<uint32_t, const char *>> formats{
			{ V4L2_PIX_FMT_YUYV, "YUYV" },
			{ 0, "<INVALID>" },
			{ v4l2_fourcc(0, 1, 2, 3), "...." },
			{ V4L2_PIX_FMT_Y16_BE, "Y16 -BE" }
		};

		for (const auto &fmt : formats) {
			if (V4L2PixelFormat(fmt.first).toString() != fmt.second) {
				cerr << "Failed to convert V4L2PixelFormat"
				     << utils::hex(fmt.first) << "to string"
				     << endl;
				return TestFail;
			}
		}

		if (V4L2PixelFormat().toString() != "<INVALID>") {
			cerr << "Failed to convert default V4L2PixelFormat to string"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(Format)
