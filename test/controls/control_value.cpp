/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_value.cpp - ControlValue tests
 */

#include <iostream>

#include <libcamera/controls.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class ControlValueTest : public Test
{
protected:
	int run()
	{
		ControlValue integer(1234);
		ControlValue boolean(true);

		/* Just a string conversion output test... no validation */
		cout << "Int: " << integer.toString()
		     << " Bool: " << boolean.toString()
		     << endl;

		if (integer.getInt() != 1234) {
			cerr << "Failed to get Integer" << endl;
			return TestFail;
		}

		if (boolean.getBool() != true) {
			cerr << "Failed to get Boolean" << endl;
			return TestFail;
		}

		/* Test an uninitialised value, and updating it. */

		ControlValue value;
		if (!value.isNone()) {
			cerr << "Empty value is non-null" << endl;
			return TestFail;
		}

		value.set(true);
		if (value.isNone()) {
			cerr << "Failed to set an empty object" << endl;
			return TestFail;
		}

		if (value.getBool() != true) {
			cerr << "Failed to get Booleans" << endl;
			return TestFail;
		}

		value.set(10);
		if (value.getInt() != 10) {
			cerr << "Failed to get Integer" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlValueTest)
