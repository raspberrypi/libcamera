/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Fixed / floating point conversions
 */

#pragma once

#include <cmath>
#include <type_traits>

namespace libcamera {

namespace ipa {

#ifndef __DOXYGEN__
template<unsigned int I, unsigned int F, typename R, typename T,
	 std::enable_if_t<std::is_integral_v<R> &&
			  std::is_floating_point_v<T>> * = nullptr>
#else
template<unsigned int I, unsigned int F, typename R, typename T>
#endif
constexpr R floatingToFixedPoint(T number)
{
	static_assert(sizeof(int) >= sizeof(R));
	static_assert(I + F <= sizeof(R) * 8);

	/*
	 * The intermediate cast to int is needed on arm platforms to properly
	 * cast negative values. See
	 * https://embeddeduse.com/2013/08/25/casting-a-negative-float-to-an-unsigned-int/
	 */
	R mask = (1 << (F + I)) - 1;
	R frac = static_cast<R>(static_cast<int>(std::round(number * (1 << F)))) & mask;

	return frac;
}

#ifndef __DOXYGEN__
template<unsigned int I, unsigned int F, typename R, typename T,
	 std::enable_if_t<std::is_floating_point_v<R> &&
			  std::is_integral_v<T>> * = nullptr>
#else
template<unsigned int I, unsigned int F, typename R, typename T>
#endif
constexpr R fixedToFloatingPoint(T number)
{
	static_assert(sizeof(int) >= sizeof(T));
	static_assert(I + F <= sizeof(T) * 8);

	/*
	 * Recreate the upper bits in case of a negative number by shifting the sign
	 * bit from the fixed point to the first bit of the unsigned and then right shifting
	 * by the same amount which keeps the sign bit in place.
	 * This can be optimized by the compiler quite well.
	 */
	int remaining_bits = sizeof(int) * 8 - (I + F);
	int t = static_cast<int>(static_cast<unsigned>(number) << remaining_bits) >> remaining_bits;
	return static_cast<R>(t) / static_cast<R>(1 << F);
}

} /* namespace ipa */

} /* namespace libcamera */
