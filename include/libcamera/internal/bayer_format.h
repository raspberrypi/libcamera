/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * bayer_format.h - Bayer Pixel Format
 */
#ifndef __LIBCAMERA_INTERNAL_BAYER_FORMAT_H__
#define __LIBCAMERA_INTERNAL_BAYER_FORMAT_H__

#include <stdint.h>
#include <string>

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
		RGGB = 3
	};

	enum Packing : uint16_t {
		None = 0,
		CSI2Packed = 1,
		IPU3Packed = 2,
	};

	constexpr BayerFormat()
		: order(Order::BGGR), bitDepth(0), packing(Packing::None)
	{
	}

	constexpr BayerFormat(Order o, uint8_t b, Packing p)
		: order(o), bitDepth(b), packing(p)
	{
	}

	explicit BayerFormat(V4L2PixelFormat v4l2Format);
	static const BayerFormat &fromMbusCode(unsigned int mbusCode);
	bool isValid() const { return bitDepth != 0; }

	std::string toString() const;

	V4L2PixelFormat toV4L2PixelFormat() const;
	static BayerFormat fromV4L2PixelFormat(V4L2PixelFormat v4l2Format);
	BayerFormat transform(Transform t) const;

	Order order;
	uint8_t bitDepth;

	Packing packing;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_BAYER_FORMAT_H__ */
