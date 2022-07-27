/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * bayer_format.h - Bayer Pixel Format
 */

#pragma once

#include <ostream>
#include <stdint.h>
#include <string>

#include <libcamera/pixel_format.h>

#include "libcamera/internal/v4l2_pixelformat.h"

namespace libcamera {

enum class Transform;

class BayerFormat
{
public:
	enum Order : uint8_t {
		BGGR = 0,
		GBRG = 1,
		GRBG = 2,
		RGGB = 3,
		MONO = 4
	};

	enum class Packing : uint16_t {
		None = 0,
		CSI2 = 1,
		IPU3 = 2,
	};

	constexpr BayerFormat()
		: order(Order::BGGR), bitDepth(0), packing(Packing::None)
	{
	}

	constexpr BayerFormat(Order o, uint8_t b, Packing p)
		: order(o), bitDepth(b), packing(p)
	{
	}

	static const BayerFormat &fromMbusCode(unsigned int mbusCode);
	bool isValid() const { return bitDepth != 0; }

	std::string toString() const;

	V4L2PixelFormat toV4L2PixelFormat() const;
	static BayerFormat fromV4L2PixelFormat(V4L2PixelFormat v4l2Format);
	PixelFormat toPixelFormat() const;
	static BayerFormat fromPixelFormat(PixelFormat format);
	BayerFormat transform(Transform t) const;

	Order order;
	uint8_t bitDepth;

	Packing packing;
};

bool operator==(const BayerFormat &lhs, const BayerFormat &rhs);
static inline bool operator!=(const BayerFormat &lhs, const BayerFormat &rhs)
{
	return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &out, const BayerFormat &f);

} /* namespace libcamera */
