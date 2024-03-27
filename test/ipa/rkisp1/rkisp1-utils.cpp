/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Miscellaneous utility tests
 */

#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>

#include "../src/ipa/rkisp1/utils.h"

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa::rkisp1;

class RkISP1UtilsTest : public Test
{
protected:
	template<unsigned int intPrec, unsigned fracPrec, typename T>
	int testSingleFixedPoint(double input, T expected)
	{
		T ret = utils::floatingToFixedPoint<intPrec, fracPrec, T>(input);
		if (ret != expected) {
			cerr << "Expected " << input << " to convert to "
			     << expected << ", got " << ret << std::endl;
			return TestFail;
		}

		/*
		 * The precision check is fairly arbitrary but is based on what
		 * the rkisp1 is capable of in the crosstalk module.
		 */
		double f = utils::fixedToFloatingPoint<intPrec, fracPrec, double>(ret);
		if (std::abs(f - input) > 0.005) {
			cerr << "Reverse conversion expected " << ret
			     << " to convert to " << input
			     << ", got " << f << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	int testFixedPoint()
	{
		/*
		 * The second 7.992 test is to test that unused bits don't
		 * affect the result.
		 */
		std::map<double, uint16_t> testCases = {
			{ 7.992, 0x3FF },
			{ 7.992, 0xBFF },
			{   0.2, 0x01A },
			{  -0.2, 0x7E6 },
			{  -0.8, 0x79A },
			{  -0.4, 0x7CD },
			{  -1.4, 0x74D },
			{    -8, 0x400 },
			{     0, 0 },
		};

		int ret;
		for (const auto &testCase : testCases) {
			ret = testSingleFixedPoint<4, 7, uint16_t>(testCase.first,
								   testCase.second);
			if (ret != TestPass)
				return ret;
		}

		return TestPass;
	}

	int run()
	{
		/* fixed point conversion test */
		if (testFixedPoint() != TestPass)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(RkISP1UtilsTest)
