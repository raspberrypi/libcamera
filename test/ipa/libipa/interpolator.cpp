/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Interpolator tests
 */

#include "../src/ipa/libipa/interpolator.h"

#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>
#include <stdio.h>

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa;

#define ASSERT_EQ(a, b)                    \
	if ((a) != (b)) {                  \
		printf(#a " != " #b "\n"); \
		return TestFail;           \
	}

class InterpolatorTest : public Test
{
protected:
	int run()
	{
		Interpolator<int> interpolator;
		interpolator.setData({ { 10, 100 }, { 20, 200 }, { 30, 300 } });

		ASSERT_EQ(interpolator.getInterpolated(0), 100);
		ASSERT_EQ(interpolator.getInterpolated(10), 100);
		ASSERT_EQ(interpolator.getInterpolated(20), 200);
		ASSERT_EQ(interpolator.getInterpolated(25), 250);
		ASSERT_EQ(interpolator.getInterpolated(30), 300);
		ASSERT_EQ(interpolator.getInterpolated(40), 300);

		interpolator.setQuantization(10);
		unsigned int q = 0;
		ASSERT_EQ(interpolator.getInterpolated(25, &q), 300);
		ASSERT_EQ(q, 30);
		ASSERT_EQ(interpolator.getInterpolated(24, &q), 200);
		ASSERT_EQ(q, 20);

		return TestPass;
	}
};

TEST_REGISTER(InterpolatorTest)
