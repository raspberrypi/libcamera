/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Histogram tests
 */

#include "../src/ipa/libipa/histogram.h"

#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa;

#define ASSERT_EQ(a, b)                                 \
	if (!((a) == (b))) {                            \
		std::cout << #a " != " #b << std::endl; \
		return TestFail;                        \
	}

class HistogramTest : public Test
{
protected:
	int run()
	{
		auto hist = Histogram({ { 50, 50 } });

		ASSERT_EQ(hist.bins(), 2);
		ASSERT_EQ(hist.total(), 100);

		ASSERT_EQ(hist.cumulativeFrequency(1.0), 50);
		ASSERT_EQ(hist.cumulativeFrequency(1.5), 75);
		ASSERT_EQ(hist.cumulativeFrequency(2.0), 100);

		ASSERT_EQ(hist.quantile(0.0), 0.0);
		ASSERT_EQ(hist.quantile(1.0), 2.0);
		ASSERT_EQ(hist.quantile(0.5), 1.0);

		/* Test quantile in the middle of a bin. */
		ASSERT_EQ(hist.quantile(0.75), 1.5);

		/* Test quantile smaller than the smallest histogram step. */
		ASSERT_EQ(hist.quantile(0.001), 0.002);

		ASSERT_EQ(hist.interQuantileMean(0.0, 1.0), 1.0);
		ASSERT_EQ(hist.interQuantileMean(0.0, 0.5), 0.5);
		ASSERT_EQ(hist.interQuantileMean(0.5, 1.0), 1.5);

		/* Test interquantile mean that starts and ends in the middle of a bin. */
		ASSERT_EQ(hist.interQuantileMean(0.25, 0.75), 1.0);

		/* Test small ranges at the borders of the histogram. */
		ASSERT_EQ(hist.interQuantileMean(0.0, 0.1), 0.1);
		ASSERT_EQ(hist.interQuantileMean(0.9, 1.0), 1.9);

		return TestPass;
	}
};

TEST_REGISTER(HistogramTest)
