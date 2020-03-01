/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_value.cpp - ControlValue tests
 */

#include <algorithm>
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

		std::array<bool, 2> bools{ true, false };
		value.set(Span<bool>(bools));
		if (value.isNone() || !value.isArray() ||
		    value.type() != ControlTypeBool) {
			cerr << "Control type mismatch after setting to bool array" << endl;
			return TestFail;
		}

		Span<const bool> boolsResult = value.get<Span<const bool>>();
		if (bools.size() != boolsResult.size() ||
		    !std::equal(bools.begin(), bools.end(), boolsResult.begin())) {
			cerr << "Control value mismatch after setting to bool" << endl;
			return TestFail;
		}

		if (value.toString() != "[ true, false ]") {
			cerr << "Control string mismatch after setting to bool array" << endl;
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

		std::array<uint8_t, 4> bytes{ 3, 14, 15, 9 };
		value.set(Span<uint8_t>(bytes));
		if (value.isNone() || !value.isArray() ||
		    value.type() != ControlTypeByte) {
			cerr << "Control type mismatch after setting to uint8_t array" << endl;
			return TestFail;
		}

		Span<const uint8_t> int8sResult = value.get<Span<const uint8_t>>();
		if (bytes.size() != int8sResult.size() ||
		    !std::equal(bytes.begin(), bytes.end(), int8sResult.begin())) {
			cerr << "Control value mismatch after setting to uint8_t array" << endl;
			return TestFail;
		}

		if (value.toString() != "[ 3, 14, 15, 9 ]") {
			cerr << "Control string mismatch after setting to uint8_t array" << endl;
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

		std::array<int32_t, 4> int32s{ 3, 14, 15, 9 };
		value.set(Span<int32_t>(int32s));
		if (value.isNone() || !value.isArray() ||
		    value.type() != ControlTypeInteger32) {
			cerr << "Control type mismatch after setting to int32_t array" << endl;
			return TestFail;
		}

		Span<const int32_t> int32sResult = value.get<Span<const int32_t>>();
		if (int32s.size() != int32sResult.size() ||
		    !std::equal(int32s.begin(), int32s.end(), int32sResult.begin())) {
			cerr << "Control value mismatch after setting to int32_t array" << endl;
			return TestFail;
		}

		if (value.toString() != "[ 3, 14, 15, 9 ]") {
			cerr << "Control string mismatch after setting to int32_t array" << endl;
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

		std::array<int64_t, 4> int64s{ 3, 14, 15, 9 };
		value.set(Span<int64_t>(int64s));
		if (value.isNone() || !value.isArray() ||
		    value.type() != ControlTypeInteger64) {
			cerr << "Control type mismatch after setting to int64_t array" << endl;
			return TestFail;
		}

		Span<const int64_t> int64sResult = value.get<Span<const int64_t>>();
		if (int64s.size() != int64sResult.size() ||
		    !std::equal(int64s.begin(), int64s.end(), int64sResult.begin())) {
			cerr << "Control value mismatch after setting to int64_t array" << endl;
			return TestFail;
		}

		if (value.toString() != "[ 3, 14, 15, 9 ]") {
			cerr << "Control string mismatch after setting to int64_t array" << endl;
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

		std::array<float, 3> floats{ 3.141593, 2.718282, 299792458.0 };
		value.set(Span<float>(floats));
		if (value.isNone() || !value.isArray() ||
		    value.type() != ControlTypeFloat) {
			cerr << "Control type mismatch after setting to float array" << endl;
			return TestFail;
		}

		Span<const float> floatsResult = value.get<Span<const float>>();
		if (floats.size() != floatsResult.size() ||
		    !std::equal(floats.begin(), floats.end(), floatsResult.begin())) {
			cerr << "Control value mismatch after setting to float array" << endl;
			return TestFail;
		}

		/*
		 * The string representation for the third value doesn't match
		 * the value in the floats array above, due to limited precision
		 * of the float type that can't properly represent the speed of
		 * light.
		 */
		if (value.toString() != "[ 3.141593, 2.718282, 299792448.000000 ]") {
			cerr << "Control string mismatch after setting to float array" << endl;
			return TestFail;
		}

		/*
		 * String type.
		 */
		std::string string{ "libcamera" };
		value.set(string);
		if (value.isNone() || !value.isArray() ||
		    value.type() != ControlTypeString ||
		    value.numElements() != string.size()) {
			cerr << "Control type mismatch after setting to string" << endl;
			return TestFail;
		}

		if (value.get<std::string>() != string) {
			cerr << "Control value mismatch after setting to string" << endl;
			return TestFail;
		}

		if (value.toString() != string) {
			cerr << "Control string mismatch after setting to string" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlValueTest)
