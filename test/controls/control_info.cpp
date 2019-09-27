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
		 * Test information retrieval from a control with no minimum
		 * and maximum.
		 */
		ControlInfo brightness(controls::Brightness);

		if (brightness.id() != controls::Brightness ||
		    brightness.id().type() != ControlTypeInteger32 ||
		    brightness.id().name() != std::string("Brightness")) {
			cout << "Invalid control identification for Brightness" << endl;
			return TestFail;
		}

		if (brightness.min().get<int32_t>() != 0 ||
		    brightness.max().get<int32_t>() != 0) {
			cout << "Invalid control range for Brightness" << endl;
			return TestFail;
		}

		/*
		 * Test information retrieval from a control with a minimum and
		 * a maximum value.
		 */
		ControlInfo contrast(controls::Contrast, 10, 200);

		if (contrast.id() != controls::Contrast ||
		    contrast.id().type() != ControlTypeInteger32 ||
		    contrast.id().name() != std::string("Contrast")) {
			cout << "Invalid control identification for Contrast" << endl;
			return TestFail;
		}

		if (contrast.min().get<int32_t>() != 10 ||
		    contrast.max().get<int32_t>() != 200) {
			cout << "Invalid control range for Contrast" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlInfoTest)
