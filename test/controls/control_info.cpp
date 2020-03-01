/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_info.cpp - ControlInfo tests
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

		if (brightness.min().get<int32_t>() != 0 ||
		    brightness.max().get<int32_t>() != 0) {
			cout << "Invalid control range for Brightness" << endl;
			return TestFail;
		}

		/*
		 * Test information retrieval from a control with a minimum and
		 * a maximum value.
		 */
		ControlInfo contrast(10, 200);

		if (contrast.min().get<int32_t>() != 10 ||
		    contrast.max().get<int32_t>() != 200) {
			cout << "Invalid control range for Contrast" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlInfoTest)
