/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.cpp - V4L2 device controls handling test
 */

#include <iostream>
#include <limits.h>

#include "v4l2_videodevice.h"

#include "v4l2_videodevice_test.h"

using namespace std;
using namespace libcamera;

class V4L2ControlTest : public V4L2VideoDeviceTest
{
public:
	V4L2ControlTest()
		: V4L2VideoDeviceTest("vivid", "vivid-000-vid-cap")
	{
	}

protected:
	int run()
	{
		const ControlInfoMap &info = capture_->controls();

		/* Test control enumeration. */
		if (info.empty()) {
			cerr << "Failed to enumerate controls" << endl;
			return TestFail;
		}

		if (info.find(V4L2_CID_BRIGHTNESS) == info.end() ||
		    info.find(V4L2_CID_CONTRAST) == info.end() ||
		    info.find(V4L2_CID_SATURATION) == info.end()) {
			cerr << "Missing controls" << endl;
			return TestFail;
		}

		const ControlRange &brightness = info.find(V4L2_CID_BRIGHTNESS)->second;
		const ControlRange &contrast = info.find(V4L2_CID_CONTRAST)->second;
		const ControlRange &saturation = info.find(V4L2_CID_SATURATION)->second;

		/* Test getting controls. */
		ControlList ctrls(info);
		ctrls.set(V4L2_CID_BRIGHTNESS, -1);
		ctrls.set(V4L2_CID_CONTRAST, -1);
		ctrls.set(V4L2_CID_SATURATION, -1);

		int ret = capture_->getControls(&ctrls);
		if (ret) {
			cerr << "Failed to get controls" << endl;
			return TestFail;
		}

		if (ctrls.get(V4L2_CID_BRIGHTNESS).get<int32_t>() == -1 ||
		    ctrls.get(V4L2_CID_CONTRAST).get<int32_t>() == -1 ||
		    ctrls.get(V4L2_CID_SATURATION).get<int32_t>() == -1) {
			cerr << "Incorrect value for retrieved controls" << endl;
			return TestFail;
		}

		/* Test setting controls. */
		ctrls.set(V4L2_CID_BRIGHTNESS, brightness.min());
		ctrls.set(V4L2_CID_CONTRAST, contrast.max());
		ctrls.set(V4L2_CID_SATURATION, saturation.min());

		ret = capture_->setControls(&ctrls);
		if (ret) {
			cerr << "Failed to set controls" << endl;
			return TestFail;
		}

		/* Test setting controls outside of range. */
		ctrls.set(V4L2_CID_BRIGHTNESS, brightness.min().get<int32_t>() - 1);
		ctrls.set(V4L2_CID_CONTRAST, contrast.max().get<int32_t>() + 1);
		ctrls.set(V4L2_CID_SATURATION, saturation.min().get<int32_t>() + 1);

		ret = capture_->setControls(&ctrls);
		if (ret) {
			cerr << "Failed to set controls (out of range)" << endl;
			return TestFail;
		}

		if (ctrls.get(V4L2_CID_BRIGHTNESS) != brightness.min() ||
		    ctrls.get(V4L2_CID_CONTRAST) != contrast.max() ||
		    ctrls.get(V4L2_CID_SATURATION) != saturation.min().get<int32_t>() + 1) {
			cerr << "Controls not updated when set" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(V4L2ControlTest);
