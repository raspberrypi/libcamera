/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Vector tests
 */

#include "libcamera/internal/vector.h"

#include <cmath>
#include <iostream>

#include "test.h"

using namespace libcamera;

#define ASSERT_EQ(a, b)							\
if ((a) != (b)) {							\
	std::cout << #a " != " #b << " (line " << __LINE__ << ")"	\
		  << std::endl;						\
	return TestFail;						\
}

class VectorTest : public Test
{
protected:
	int run()
	{
		Vector<double, 3> v1{ 0.0 };

		ASSERT_EQ(v1[0], 0.0);
		ASSERT_EQ(v1[1], 0.0);
		ASSERT_EQ(v1[2], 0.0);

		ASSERT_EQ(v1.length(), 0.0);
		ASSERT_EQ(v1.length2(), 0.0);

		Vector<double, 3> v2{{ 1.0, 4.0, 8.0 }};

		ASSERT_EQ(v2[0], 1.0);
		ASSERT_EQ(v2[1], 4.0);
		ASSERT_EQ(v2[2], 8.0);

		ASSERT_EQ(v2.x(), 1.0);
		ASSERT_EQ(v2.y(), 4.0);
		ASSERT_EQ(v2.z(), 8.0);

		ASSERT_EQ(v2.r(), 1.0);
		ASSERT_EQ(v2.g(), 4.0);
		ASSERT_EQ(v2.b(), 8.0);

		ASSERT_EQ(v2.length2(), 81.0);
		ASSERT_EQ(v2.length(), 9.0);
		ASSERT_EQ(v2.sum(), 13.0);

		Vector<double, 3> v3{ v2 };

		ASSERT_EQ(v2, v3);

		v3 = Vector<double, 3>{{ 4.0, 4.0, 4.0 }};

		ASSERT_EQ(v2 + v3, (Vector<double, 3>{{ 5.0, 8.0, 12.0 }}));
		ASSERT_EQ(v2 + 4.0, (Vector<double, 3>{{ 5.0, 8.0, 12.0 }}));
		ASSERT_EQ(v2 - v3, (Vector<double, 3>{{ -3.0, 0.0, 4.0 }}));
		ASSERT_EQ(v2 - 4.0, (Vector<double, 3>{{ -3.0, 0.0, 4.0 }}));
		ASSERT_EQ(v2 * v3, (Vector<double, 3>{{ 4.0, 16.0, 32.0 }}));
		ASSERT_EQ(v2 * 4.0, (Vector<double, 3>{{ 4.0, 16.0, 32.0 }}));
		ASSERT_EQ(v2 / v3, (Vector<double, 3>{{ 0.25, 1.0, 2.0 }}));
		ASSERT_EQ(v2 / 4.0, (Vector<double, 3>{{ 0.25, 1.0, 2.0 }}));

		ASSERT_EQ(v2.min(v3), (Vector<double, 3>{{ 1.0, 4.0, 4.0 }}));
		ASSERT_EQ(v2.min(4.0), (Vector<double, 3>{{ 1.0, 4.0, 4.0 }}));
		ASSERT_EQ(v2.max(v3), (Vector<double, 3>{{ 4.0, 4.0, 8.0 }}));
		ASSERT_EQ(v2.max(4.0), (Vector<double, 3>{{ 4.0, 4.0, 8.0 }}));

		ASSERT_EQ(v2.dot(v3), 52.0);

		v2 += v3;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 5.0, 8.0, 12.0 }}));
		v2 -= v3;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 1.0, 4.0, 8.0 }}));
		v2 *= v3;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 4.0, 16.0, 32.0 }}));
		v2 /= v3;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 1.0, 4.0, 8.0 }}));

		v2 += 4.0;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 5.0, 8.0, 12.0 }}));
		v2 -= 4.0;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 1.0, 4.0, 8.0 }}));
		v2 *= 4.0;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 4.0, 16.0, 32.0 }}));
		v2 /= 4.0;
		ASSERT_EQ(v2, (Vector<double, 3>{{ 1.0, 4.0, 8.0 }}));

		return TestPass;
	}
};

TEST_REGISTER(VectorTest)
