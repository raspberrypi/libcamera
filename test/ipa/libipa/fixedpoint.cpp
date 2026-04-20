/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Fixed / Floating point utility tests
 */

#include "../src/ipa/libipa/fixedpoint.h"

#include <cmath>
#include <iostream>
#include <map>
#include <stdint.h>

#include <libcamera/base/utils.h>

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa;

class FixedPointUtilsTest : public Test
{
protected:
	template<typename Q>
	int quantizedCheck(float input, typename Q::QuantizedType expected, float value)
	{
		Q q(input);
		using T = typename Q::QuantizedType;

		std::cout << "  Checking " << input << " == " << q << std::endl;

		T quantized = q.quantized();
		if (quantized != expected) {
			std::cout << "    ** Q Expected " << input
				  << " to quantize to " << utils::hex(expected)
				  << ", got " << utils::hex(quantized)
				  << " - (" << q << ")"
				  << std::endl;
			return 1;
		}

		if ((std::abs(q.value() - value)) > 0.0001f) {
			std::cout << "    ** V Expected " << input
				  << " to quantize to " << value
				  << ", got " << q.value()
				  << " - (" << q << ")"
				  << std::endl;
			return 1;
		}

		return 0;
	}

	template<typename Q, typename RegisterType>
	int signExtendCheck()
	{
		Q minValue(Q::TraitsType::min);
		using T = typename Q::QuantizedType;
		T minQuantized = minValue.quantized();

		/* Take the value and expand it into a larger register */
		RegisterType reg = static_cast<RegisterType>(minValue.quantized());

		std::cout << "  Checking " << minValue << " == " << utils::hex(reg) << std::endl;

		/*
		 * Ensure that the minimum negative value is not sign
		 * extended when casting up to a larger register storage
		 * type.
		 */
		if (reg != minQuantized) {
			std::cout << "    ** Sign extension check failed for min value. Expected "
				  << utils::hex(minQuantized) << ", got "
				  << utils::hex(reg) << std::endl;
			return 1;
		}

		/* And we should be consistent when we convert back to Q */
		Q q2(static_cast<T>(reg));
		if (!(q2 == minValue)) {
			std::cout << "    ** Sign extension check failed for min value. Expected "
				  << minValue << ", got " << q2 << std::endl;
			return 1;
		}

		return 0;
	}

	template<typename Q>
	void introduce(std::string_view type)
	{
		using T = typename Q::QuantizedType;

		std::cout << std::endl;

		std::cout << type
			  << "(" << Q::TraitsType::min << " .. " << Q::TraitsType::max << ") "
			  << " Min: " << Q(Q::TraitsType::min)
			  << " -- Max: " << Q(Q::TraitsType::max)
			  << " Step:" << Q(T(1)).value()
			  << std::endl;
	}

	int testFixedPointQuantizers()
	{
		unsigned int fails = 0;

		/*
		 * These aim to specifically test all the corner cases of the
		 * quantization and de-quantization process. Including clamping
		 * to min/max, zero points and making sure that steps are
		 * correct.
		 *
		 * In particular test signed and unsigned types and a mix of the
		 * highest bit width of a storage type and smaller widths that
		 * require bit masking and sign extension.
		 *
		 * Note we must hard code these. Any calculation of expected
		 * values risks replicating bugs in the implementation.
		 *
		 * As the underlying types are integer and float the limit of
		 * precision is around 24 bits so we do not test wider types.
		 */

		/* clang-format off */

		/* Q1.7(-1 .. 0.992188)  Min: [0x80:-1] -- Max: [0x7f:0.992188] Step:0.0078125*/
		introduce<Q<1, 7>>("Q1.7");
		fails += quantizedCheck<Q<1, 7>>(-2.000f, 0b1'0000000, -1.0f);		/* Clamped to Min */
		fails += quantizedCheck<Q<1, 7>>(-1.000f, 0b1'0000000, -1.0f);		/* Min */
		fails += quantizedCheck<Q<1, 7>>(-0.992f, 0b1'0000001, -0.992188f);	/* Min + 1 step */
		fails += quantizedCheck<Q<1, 7>>(-0.006f, 0b1'1111111, -0.0078125f);	/* -1 step */
		fails += quantizedCheck<Q<1, 7>>( 0.000f, 0b0'0000000,  0.0f);		/* Zero */
		fails += quantizedCheck<Q<1, 7>>( 0.008f, 0b0'0000001,  0.0078125f);	/* +1 step */
		fails += quantizedCheck<Q<1, 7>>( 0.992f, 0b0'1111111,  0.992188f);	/* Max */
		fails += quantizedCheck<Q<1, 7>>( 2.000f, 0b0'1111111,  0.992188f);	/* Clamped to Max */

		/* UQ1.7(0 .. 1.99219)  Min: [0x00:0] -- Max: [0xff:1.99219] Step:0.0078125 */
		introduce<UQ<1, 7>>("UQ1.7");
		fails += quantizedCheck<UQ<1, 7>>(-1.0f,   0b0'0000000, 0.0f);		/* Clamped to Min */
		fails += quantizedCheck<UQ<1, 7>>( 0.0f,   0b0'0000000, 0.0f);		/* Min / Zero */
		fails += quantizedCheck<UQ<1, 7>>( 1.0f,   0b1'0000000, 1.0f);		/* Mid */
		fails += quantizedCheck<UQ<1, 7>>( 1.992f, 0b1'1111111, 1.99219f);	/* Max */
		fails += quantizedCheck<UQ<1, 7>>( 2.000f, 0b1'1111111, 1.99219f);	/* Clamped to Max */

		/* Q4.7(-8 .. 7.99219)  Min: [0x0400:-8] -- Max: [0x03ff:7.99219] Step:0.0078125 */
		introduce<Q<4, 7>>("Q4.7");
		fails += quantizedCheck<Q<4, 7>>(-8.0f,   0b1000'0000000, -8.0f);	/* Min */
		fails += quantizedCheck<Q<4, 7>>(-0.008f, 0b1111'1111111, -0.0078125);	/* -1 step */
		fails += quantizedCheck<Q<4, 7>>( 0.0f,   0b0000'0000000,  0.0f);	/* Zero */
		fails += quantizedCheck<Q<4, 7>>( 0.008f, 0b0000'0000001,  0.0078125f);	/* +1 step */
		fails += quantizedCheck<Q<4, 7>>( 7.992f, 0b0111'1111111,  7.99219f);	/* Max */

		/* Retain additional tests from original testFixedPoint() */
		fails += quantizedCheck<Q<4, 7>>( 0.2f, 0b0000'0011010,  0.203125f);	/* 0x01a */
		fails += quantizedCheck<Q<4, 7>>(-0.2f, 0b1111'1100110, -0.203125f);	/* 0x7e6 */
		fails += quantizedCheck<Q<4, 7>>(-0.8f, 0b1111'0011010, -0.796875f);	/* 0x79a */
		fails += quantizedCheck<Q<4, 7>>(-0.4f, 0b1111'1001101, -0.398438f);	/* 0x7cd */
		fails += quantizedCheck<Q<4, 7>>(-1.4f, 0b1110'1001101, -1.39844f);	/* 0x74d */

		/* UQ4.8(0 .. 15.9961)  Min: [0x0000:0] -- Max: [0x0fff:15.9961] Step:0.00390625 */
		introduce<UQ<4, 8>>("UQ4.8");
		fails += quantizedCheck<UQ<4, 8>>( 0.0f, 0b0000'00000000,  0.00f);
		fails += quantizedCheck<UQ<4, 8>>(16.0f, 0b1111'11111111, 15.9961f);

		/* Q5.4(-16 .. 15.9375)  Min: [0x0100:-16] -- Max: [0x00ff:15.9375] Step:0.0625 */
		introduce<Q<5, 4>>("Q5.4");
		fails += quantizedCheck<Q<5, 4>>(-16.00f, 0b10000'0000, -16.00f);
		fails += quantizedCheck<Q<5, 4>>( 15.94f, 0b01111'1111,  15.9375f);

		/* UQ5.8(0 .. 31.9961)  Min: [0x0000:0] -- Max: [0x1fff:31.9961] Step:0.00390625 */
		introduce<UQ<5, 8>>("UQ5.8");
		fails += quantizedCheck<UQ<5, 8>>( 0.00f, 0b00000'00000000,  0.00f);
		fails += quantizedCheck<UQ<5, 8>>(32.00f, 0b11111'11111111, 31.9961f);

		/* Q12.4(-2048 .. 2047.94)  Min: [0x8000:-2048] -- Max: [0x7fff:2047.94] Step:0.0625 */
		introduce<Q<12, 4>>("Q12.4");
		fails += quantizedCheck<Q<12, 4>>(    0.0f, 0b000000000000'0000,     0.0f);
		fails += quantizedCheck<Q<12, 4>>(    7.5f, 0b000000000111'1000,     7.5f);

		/* UQ12.4(0 .. 4095.94)  Min: [0x0000:0] -- Max: [0xffff:4095.94] Step:0.0625 */
		introduce<UQ<12, 4>>("UQ12.4");
		fails += quantizedCheck<UQ<12, 4>>(0.0f, 0b000000000000'0000, 0.0f);
		fails += quantizedCheck<UQ<12, 4>>(7.5f, 0b000000000111'1000, 7.5f);

		/* Q4.20 */
		introduce<Q<4, 20>>("Q4.20");
		fails += quantizedCheck<Q<4, 20>>( -9.0f, 0b1000'00000000000000000000, -8.0f);
		fails += quantizedCheck<Q<4, 20>>( -8.0f, 0b1000'00000000000000000000, -8.0f);
		fails += quantizedCheck<Q<4, 20>>(  8.0f, 0b0111'11111111111111111111,  8.0f);
		fails += quantizedCheck<Q<4, 20>>(  9.0f, 0b0111'11111111111111111111,  8.0f);

		/* UQ4.20 */
		introduce<UQ<4, 20>>("UQ4.20");
		fails += quantizedCheck<UQ<4, 20>>(-1.0f, 0b0000'00000000000000000000,  0.0f);
		fails += quantizedCheck<UQ<4, 20>>( 0.0f, 0b0000'00000000000000000000,  0.0f);
		fails += quantizedCheck<UQ<4, 20>>(16.0f, 0b1111'11111111111111111111, 16.0f);
		fails += quantizedCheck<UQ<4, 20>>(20.0f, 0b1111'11111111111111111111, 16.0f);

		/* Storage selection tests */
		static_assert(std::is_same_v<typename Q<4, 4>::TraitsType::QuantizedType, uint8_t>);
		static_assert(std::is_same_v<typename UQ<4, 4>::TraitsType::QuantizedType, uint8_t>);
		static_assert(std::is_same_v<typename Q<8, 8>::TraitsType::QuantizedType, uint16_t>);
		static_assert(std::is_same_v<typename UQ<8, 8>::TraitsType::QuantizedType, uint16_t>);
		static_assert(std::is_same_v<typename Q<20, 4>::TraitsType::QuantizedType, uint32_t>);
		static_assert(std::is_same_v<typename UQ<20, 4>::TraitsType::QuantizedType, uint32_t>);

		/*
		 * Test and validate that sign extension can not modify a
		 * quantized value when stored or cast to a larger register.
		 */
		std::cout << std::endl;
		std::cout << "Testing sign extension of quantized values when cast to larger registers" << std::endl;
		fails += signExtendCheck<Q<2, 4>, uint8_t>();
		fails += signExtendCheck<Q<4, 4>, uint8_t>();
		fails += signExtendCheck<Q<4, 4>, uint16_t>();
		fails += signExtendCheck<Q<8, 8>, uint16_t>();
		fails += signExtendCheck<Q<8, 8>, uint32_t>();

		/* clang-format on */

		std::cout << std::endl;

		if (fails > 0) {
			cout << "Fixed point quantizer tests failed: "
			     << std::dec << fails << " failures." << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	int run()
	{
		unsigned int fails = 0;

		if (testFixedPointQuantizers() != TestPass)
			fails++;

		return fails ? TestFail : TestPass;
	}
};

TEST_REGISTER(FixedPointUtilsTest)
