/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera V4L2 Subdevice format handling test
 */

#include <iostream>
#include <limits.h>

#include "libcamera/internal/v4l2_subdevice.h"

#include "v4l2_subdevice_test.h"

using namespace std;
using namespace libcamera;

/* Test format handling on the "Scaler" subdevice of vimc media device.  */

class FormatHandlingTest : public V4L2SubdeviceTest
{
protected:
	int run() override;
};

int FormatHandlingTest::run()
{
	V4L2SubdeviceFormat format = {};

	/*
	 * Get format on a non-existing Scaler pad: expect failure.
	 */
	int ret = scaler_->getFormat(2, &format);
	if (!ret) {
		cerr << "Getting format on a non existing pad should fail" << endl;
		return TestFail;
	}

	ret = scaler_->getFormat(0, &format);
	if (ret) {
		cerr << "Failed to get format" << endl;
		return TestFail;
	}

	/*
	 * Set unrealistic image resolutions and make sure it gets updated.
	 */
	format.size = { UINT_MAX, UINT_MAX };
	ret = scaler_->setFormat(0, &format);
	if (ret) {
		cerr << "Failed to set format: image resolution is wrong, but "
		     << "setFormat() should not fail." << endl;
		return TestFail;
	}

	if (format.size.width == UINT_MAX ||
	    format.size.height == UINT_MAX) {
		cerr << "Failed to update image format" << endl;
		return TestFail;
	}

	format.size = { 0, 0 };
	ret = scaler_->setFormat(0, &format);
	if (ret) {
		cerr << "Failed to set format: image resolution is wrong, but "
		     << "setFormat() should not fail." << endl;
		return TestFail;
	}

	if (format.size.isNull()) {
		cerr << "Failed to update image format" << endl;
		return TestFail;
	}

	return TestPass;
}

TEST_REGISTER(FormatHandlingTest)
