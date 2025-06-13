/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * PWL tests
 */

#include "../src/ipa/libipa/pwl.h"

#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>
#include <stdio.h>

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa;

#define ASSERT_EQ(a, b)                                                 \
	if ((a) != (b)) {                                               \
		std::cout << #a " != " #b " (" << a << " ," << b << ")" \
			  << std::endl;                                 \
		return TestFail;                                        \
	}

class PwlTest : public Test
{
protected:
	int run()
	{
		Pwl pwl;
		pwl.append(0, 0);
		pwl.append(1, 1);

		ASSERT_EQ(pwl.eval(-1), -1);
		ASSERT_EQ(pwl.eval(0), 0);
		ASSERT_EQ(pwl.eval(0.5), 0.5);
		ASSERT_EQ(pwl.eval(1), 1);
		ASSERT_EQ(pwl.eval(2), 2);

		ASSERT_EQ(pwl.size(), 2);

		return TestPass;
	}
};

TEST_REGISTER(PwlTest)
