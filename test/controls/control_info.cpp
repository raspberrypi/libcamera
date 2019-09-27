/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_info.cpp - ControlInfo tests
 */

#include <iostream>

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
		ControlInfo info(Brightness);

		if (info.id() != Brightness ||
		    info.type() != ControlTypeInteger32 ||
		    info.name() != std::string("Brightness")) {
			cout << "Invalid control identification for Brightness" << endl;
			return TestFail;
		}

		if (info.min().get<int32_t>() != 0 ||
		    info.max().get<int32_t>() != 0) {
			cout << "Invalid control range for Brightness" << endl;
			return TestFail;
		}

		/*
		 * Test information retrieval from a control with a minimum and
		 * a maximum value.
		 */
		info = ControlInfo(Contrast, 10, 200);

		if (info.id() != Contrast ||
		    info.type() != ControlTypeInteger32 ||
		    info.name() != std::string("Contrast")) {
			cout << "Invalid control identification for Contrast" << endl;
			return TestFail;
		}

		if (info.min().get<int32_t>() != 10 ||
		    info.max().get<int32_t>() != 200) {
			cout << "Invalid control range for Contrast" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlInfoTest)
