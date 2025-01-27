/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Fixed / Floating point utility tests
 */

#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>

#include "../src/ipa/libipa/fixedpoint.h"

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa;

class FixedPointUtilsTest : public Test
{
protected:
	/* R for real, I for integer */
	template<unsigned int IntPrec, unsigned int FracPrec, typename I, typename R>
	int testFixedToFloat(I input, R expected)
	{
		R out = fixedToFloatingPoint<IntPrec, FracPrec, R>(input);
		R prec = 1.0 / (1 << FracPrec);
		if (std::abs(out - expected) > prec) {
			cerr << "Reverse conversion expected " << input
			     << " to convert to " << expected
			     << ", got " << out << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	template<unsigned int IntPrec, unsigned int FracPrec, typename T>
	int testSingleFixedPoint(double input, T expected)
	{
		T ret = floatingToFixedPoint<IntPrec, FracPrec, T>(input);
		if (ret != expected) {
			cerr << "Expected " << input << " to convert to "
			     << expected << ", got " << ret << std::endl;
			return TestFail;
		}

		/*
		 * The precision check is fairly arbitrary but is based on what
		 * the rkisp1 is capable of in the crosstalk module.
		 */
		double f = fixedToFloatingPoint<IntPrec, FracPrec, double>(ret);
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
			{ 7.992, 0x3ff },
			{   0.2, 0x01a },
			{  -0.2, 0x7e6 },
			{  -0.8, 0x79a },
			{  -0.4, 0x7cd },
			{  -1.4, 0x74d },
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

		/* Special case with a superfluous one in the unused bits */
		ret = testFixedToFloat<4, 7, uint16_t, double>(0xbff, 7.992);
		if (ret != TestPass)
			return ret;

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

TEST_REGISTER(FixedPointUtilsTest)
