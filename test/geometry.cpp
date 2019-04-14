/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.cpp - Geometry classes tests
 */

#include <iostream>

#include <libcamera/geometry.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class GeometryTest : public Test
{
protected:
	bool compare(const Size &lhs, const Size &rhs,
		     bool (*op)(const Size &lhs, const Size &rhs),
		     const char *opName, bool expect)
	{
		bool result = op(lhs, rhs);

		if (result != expect) {
			cout << "Size(" << lhs.width << ", " << lhs.height << ") "
			     << opName << " "
			     << "Size(" << rhs.width << ", " << rhs.height << ") "
			     << "test failed" << std::endl;
			return false;
		}

		return true;
	}

	int run()
	{
		/* Test Size equality and inequality. */
		if (!compare(Size(100, 100), Size(100, 100), &operator==, "==", true))
			return TestFail;
		if (!compare(Size(100, 100), Size(100, 100), &operator!=, "!=", false))
			return TestFail;

		if (!compare(Size(100, 100), Size(200, 100), &operator==, "==", false))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 100), &operator!=, "!=", true))
			return TestFail;

		if (!compare(Size(100, 100), Size(100, 200), &operator==, "==", false))
			return TestFail;
		if (!compare(Size(100, 100), Size(100, 200), &operator!=, "!=", true))
			return TestFail;

		/* Test Size ordering based on combined with and height. */
		if (!compare(Size(100, 100), Size(200, 200), &operator<, "<", true))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 200), &operator<=, "<=", true))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 200), &operator>, ">", false))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 200), &operator>=, ">=", false))
			return TestFail;

		if (!compare(Size(200, 200), Size(100, 100), &operator<, "<", false))
			return TestFail;
		if (!compare(Size(200, 200), Size(100, 100), &operator<=, "<=", false))
			return TestFail;
		if (!compare(Size(200, 200), Size(100, 100), &operator>, ">", true))
			return TestFail;
		if (!compare(Size(200, 200), Size(100, 100), &operator>=, ">=", true))
			return TestFail;

		/* Test Size ordering based on area (with overlapping sizes). */
		if (!compare(Size(200, 100), Size(100, 400), &operator<, "<", true))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 400), &operator<=, "<=", true))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 400), &operator>, ">", false))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 400), &operator>=, ">=", false))
			return TestFail;

		if (!compare(Size(100, 400), Size(200, 100), &operator<, "<", false))
			return TestFail;
		if (!compare(Size(100, 400), Size(200, 100), &operator<=, "<=", false))
			return TestFail;
		if (!compare(Size(100, 400), Size(200, 100), &operator>, ">", true))
			return TestFail;
		if (!compare(Size(100, 400), Size(200, 100), &operator>=, ">=", true))
			return TestFail;

		/* Test Size ordering based on width (with identical areas). */
		if (!compare(Size(100, 200), Size(200, 100), &operator<, "<", true))
			return TestFail;
		if (!compare(Size(100, 200), Size(200, 100), &operator<=, "<=", true))
			return TestFail;
		if (!compare(Size(100, 200), Size(200, 100), &operator>, ">", false))
			return TestFail;
		if (!compare(Size(100, 200), Size(200, 100), &operator>=, ">=", false))
			return TestFail;

		if (!compare(Size(200, 100), Size(100, 200), &operator<, "<", false))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 200), &operator<=, "<=", false))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 200), &operator>, ">", true))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 200), &operator>=, ">=", true))
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(GeometryTest)
