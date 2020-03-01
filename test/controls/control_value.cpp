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
		/*
		 * None type.
		 */
		ControlValue value;
		if (!value.isNone() || value.isArray()) {
			cerr << "Empty value is non-null" << endl;
			return TestFail;
		}

		/*
		 * Bool type.
		 */
		value.set(true);
		if (value.isNone() || value.isArray() ||
		    value.type() != ControlTypeBool) {
			cerr << "Control type mismatch after setting to bool" << endl;
			return TestFail;
		}

		if (value.get<bool>() != true) {
			cerr << "Control value mismatch after setting to bool" << endl;
			return TestFail;
		}

		if (value.toString() != "true") {
			cerr << "Control string mismatch after setting to bool" << endl;
			return TestFail;
		}

		/*
		 * Integer8 type.
		 */
		value.set(static_cast<uint8_t>(42));
		if (value.isNone() || value.isArray() ||
		    value.type() != ControlTypeByte) {
			cerr << "Control type mismatch after setting to uint8_t" << endl;
			return TestFail;
		}

		if (value.get<uint8_t>() != 42) {
			cerr << "Control value mismatch after setting to uint8_t" << endl;
			return TestFail;
		}

		if (value.toString() != "42") {
			cerr << "Control string mismatch after setting to uint8_t" << endl;
			return TestFail;
		}

		/*
		 * Integer32 type.
		 */
		value.set(0x42000000);
		if (value.isNone() || value.isArray() ||
		    value.type() != ControlTypeInteger32) {
			cerr << "Control type mismatch after setting to int32_t" << endl;
			return TestFail;
		}

		if (value.get<int32_t>() != 0x42000000) {
			cerr << "Control value mismatch after setting to int32_t" << endl;
			return TestFail;
		}

		if (value.toString() != "1107296256") {
			cerr << "Control string mismatch after setting to int32_t" << endl;
			return TestFail;
		}

		/*
		 * Integer64 type.
		 */
		value.set(static_cast<int64_t>(-42));
		if (value.isNone() || value.isArray() ||
		    value.type() != ControlTypeInteger64) {
			cerr << "Control type mismatch after setting to int64_t" << endl;
			return TestFail;
		}

		if (value.get<int64_t>() != -42) {
			cerr << "Control value mismatch after setting to int64_t" << endl;
			return TestFail;
		}

		if (value.toString() != "-42") {
			cerr << "Control string mismatch after setting to int64_t" << endl;
			return TestFail;
		}

		/*
		 * Float type.
		 */
		value.set(-0.42f);
		if (value.isNone() || value.isArray() ||
		    value.type() != ControlTypeFloat) {
			cerr << "Control type mismatch after setting to float" << endl;
			return TestFail;
		}

		if (value.get<float>() != -0.42f) {
			cerr << "Control value mismatch after setting to float" << endl;
			return TestFail;
		}

		if (value.toString() != "-0.420000") {
			cerr << "Control string mismatch after setting to float" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlValueTest)
