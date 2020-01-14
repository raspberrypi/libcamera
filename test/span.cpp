/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * span.cpp - Span tests
 */

/*
 * Include first to ensure the header is self-contained, as there's no span.cpp
 * in libcamera.
 */
#include <libcamera/span.h>

#include <array>
#include <iostream>
#include <vector>

#include "test.h"

using namespace std;
using namespace libcamera;

class SpanTest : public Test
{
protected:
	int run()
	{
		int i[4]{ 1, 2, 3, 4 };
		std::array<int, 4> a{ 1, 2, 3, 4 };
		const std::array<int, 4> ca{ 1, 2, 3, 4 };
		std::vector<int> v{ 1, 2, 3, 4 };
		const std::vector<int> cv{ 1, 2, 3, 4 };

		/*
		 * Compile-test construction and usage of spans with static
		 * extent. Commented-out tests are expected not to compile, or
		 * to generate undefined behaviour.
		 */

		Span<int, 0>{};
		/* Span<int, 4>{}; */

		Span<int, 4>{ &i[0], 4 };
		Span<int, 4>{ &i[0], &i[3] };

		Span<int, 4>{ i };
		/* Span<float, 4>{ i }; */
		/* Span<int, 2>{ i }; */

		Span<int, 4>{ a };
		Span<const int, 4>{ a };
		/* Span<float, 4>{ a }; */
		/* Span<int, 2>{ a }; */

		Span<const int, 4>{ ca };
		/* Span<const int, 2>{ ca }; */
		/* Span<const float, 4>{ ca }; */
		/* Span<int, 4>{ ca }; */

		Span<int, 4>{ v };
		Span<const int, 4>{ v };
		/* Span<float, 4>{ v }; */

		Span<const int, 4>{ v };
		/* Span<int, 4>{ v }; */
		/* Span<const float, 4>{ v }; */

		Span<int, 4> staticSpan{ i };
		Span<int, 4>{ staticSpan };
		Span<const int, 4>{ staticSpan };
		/* Span<const int, 2>{ staticSpan }; */

		staticSpan = Span<int, 4>{ v };

		staticSpan.begin();
		staticSpan.cbegin();
		staticSpan.end();
		staticSpan.cend();
		staticSpan.rbegin();
		staticSpan.crbegin();
		staticSpan.rend();
		staticSpan.crend();

		staticSpan.front();
		staticSpan.back();
		staticSpan[0];
		staticSpan.data();

		staticSpan.size();
		staticSpan.size_bytes();

		staticSpan.empty();

		staticSpan.first<2>();
		staticSpan.first(2);
		/* staticSpan.first<6>(); */
		/* staticSpan.first(6); */
		staticSpan.last<2>();
		staticSpan.last(2);
		/* staticSpan.last<6>(); */
		/* staticSpan.last(6); */
		staticSpan.subspan<1>();
		staticSpan.subspan<1, 2>();
		staticSpan.subspan(1);
		staticSpan.subspan(1, 2);
		/* staticSpan.subspan(2, 4); */

		/*
		 * Compile-test construction and usage of spans with static
		 * extent. Commented-out tests are expected not to compile, or
		 * to generate undefined behaviour.
		 */

		Span<int>{};

		Span<int>{ &i[0], 4 };
		Span<int>{ &i[0], &i[3] };

		Span<int>{ i };
		/* Span<float>{ i }; */

		Span<int>{ a };
		Span<const int>{ a };
		/* Span<float>{ a }; */

		Span<const int>{ ca };
		/* Span<const float>{ca}; */
		/* Span<int>{ca}; */

		Span<int>{ v };
		Span<const int>{ v };
		/* Span<float>{ v }; */

		Span<const int>{ v };
		/* Span<int>{ v }; */
		/* Span<const float>{ v }; */

		Span<int> dynamicSpan{ i };
		Span<int>{ dynamicSpan };
		Span<const int>{ dynamicSpan };

		dynamicSpan = Span<int>{ a };

		dynamicSpan.begin();
		dynamicSpan.cbegin();
		dynamicSpan.end();
		dynamicSpan.cend();
		dynamicSpan.rbegin();
		dynamicSpan.crbegin();
		dynamicSpan.rend();
		dynamicSpan.crend();

		dynamicSpan.front();
		dynamicSpan.back();
		dynamicSpan[0];
		dynamicSpan.data();

		dynamicSpan.size();
		dynamicSpan.size_bytes();

		dynamicSpan.empty();

		dynamicSpan.first<2>();
		dynamicSpan.first(2);
		/* dynamicSpan.first<6>(); */
		/* dynamicSpan.first(6); */
		dynamicSpan.last<2>();
		dynamicSpan.last(2);
		/* dynamicSpan.last<6>(); */
		/* dynamicSpan.last(6); */
		dynamicSpan.subspan<1>();
		dynamicSpan.subspan<1, 2>();
		dynamicSpan.subspan(1);
		dynamicSpan.subspan(1, 2);
		/* dynamicSpan.subspan(2, 4); */

		return TestPass;
	}
};

TEST_REGISTER(SpanTest)
