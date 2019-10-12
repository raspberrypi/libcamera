/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.cpp - V4L2 device controls handling test
 */

#include <climits>
#include <iostream>

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
		const V4L2ControlInfoMap &info = capture_->controls();

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

		const V4L2ControlInfo &brightness = info.find(V4L2_CID_BRIGHTNESS)->second;
		const V4L2ControlInfo &contrast = info.find(V4L2_CID_CONTRAST)->second;
		const V4L2ControlInfo &saturation = info.find(V4L2_CID_SATURATION)->second;

		/* Test getting controls. */
		V4L2ControlList ctrls;
		ctrls.add(V4L2_CID_BRIGHTNESS);
		ctrls.add(V4L2_CID_CONTRAST);
		ctrls.add(V4L2_CID_SATURATION);

		int ret = capture_->getControls(&ctrls);
		if (ret) {
			cerr << "Failed to get controls" << endl;
			return TestFail;
		}

		/* Test setting controls. */
		ctrls[V4L2_CID_BRIGHTNESS]->value() = brightness.range().min();
		ctrls[V4L2_CID_CONTRAST]->value() = contrast.range().max();
		ctrls[V4L2_CID_SATURATION]->value() = saturation.range().min();

		ret = capture_->setControls(&ctrls);
		if (ret) {
			cerr << "Failed to set controls" << endl;
			return TestFail;
		}

		/* Test setting controls outside of range. */
		ctrls[V4L2_CID_BRIGHTNESS]->value() = brightness.range().min().get<int32_t>() - 1;
		ctrls[V4L2_CID_CONTRAST]->value() = contrast.range().max().get<int32_t>() + 1;
		ctrls[V4L2_CID_SATURATION]->value() = saturation.range().min().get<int32_t>() + 1;

		ret = capture_->setControls(&ctrls);
		if (ret) {
			cerr << "Failed to set controls (out of range)" << endl;
			return TestFail;
		}

		if (ctrls[V4L2_CID_BRIGHTNESS]->value() != brightness.range().min() ||
		    ctrls[V4L2_CID_CONTRAST]->value() != brightness.range().max() ||
		    ctrls[V4L2_CID_SATURATION]->value() != saturation.range().min().get<int32_t>() + 1) {
			cerr << "Controls not updated when set" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(V4L2ControlTest);
