/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ControlInfo tests
 */

#include <iostream>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class ControlInfoTest : public Test
{
protected:
	int run()
	{
		/*
		 * Test information retrieval from a range with no minimum and
		 * maximum.
		 */
		ControlInfo brightness;

		if (brightness.min().type() != ControlType::ControlTypeNone ||
		    brightness.max().type() != ControlType::ControlTypeNone ||
		    brightness.def().type() != ControlType::ControlTypeNone) {
			cout << "Invalid control range for Brightness" << endl;
			return TestFail;
		}

		/*
		 * Test information retrieval from a control with a minimum and
		 * a maximum value, and an implicit default value.
		 */
		ControlInfo contrast(10, 200);

		if (contrast.min().get<int32_t>() != 10 ||
		    contrast.max().get<int32_t>() != 200 ||
		    !contrast.def().isNone()) {
			cout << "Invalid control range for Contrast" << endl;
			return TestFail;
		}

		/*
		 * Test information retrieval from a control with boolean
		 * values.
		 */
		ControlInfo aeEnable(false, true, false);

		if (aeEnable.min().get<bool>() != false ||
		    aeEnable.def().get<bool>() != false ||
		    aeEnable.max().get<bool>() != true) {
			cout << "Invalid control range for AeEnable" << endl;
			return TestFail;
		}

		if (!aeEnable.values().empty()) {
			cout << "Invalid control values for AeEnable" << endl;
			return TestFail;
		}

		ControlInfo awbEnable(true, true, true);

		if (awbEnable.min().get<bool>() != true ||
		    awbEnable.def().get<bool>() != true ||
		    awbEnable.max().get<bool>() != true) {
			cout << "Invalid control range for AwbEnable" << endl;
			return TestFail;
		}

		if (!awbEnable.values().empty()) {
			cout << "Invalid control values for AwbEnable" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlInfoTest)
