/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * v4l2_pixelformat.h - V4L2 Pixel Format
 */

#pragma once

#include <functional>
#include <ostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include <libcamera/pixel_format.h>

namespace libcamera {

class V4L2PixelFormat
{
public:
	struct Info {
		PixelFormat format;
		const char *description;
	};

	V4L2PixelFormat()
		: fourcc_(0)
	{
	}

	explicit V4L2PixelFormat(uint32_t fourcc)
		: fourcc_(fourcc)
	{
	}

	bool isValid() const { return fourcc_ != 0; }
	uint32_t fourcc() const { return fourcc_; }
	operator uint32_t() const { return fourcc_; }

	std::string toString() const;
	const char *description() const;

	PixelFormat toPixelFormat(bool warn = true) const;
	static const std::vector<V4L2PixelFormat> &
	fromPixelFormat(const PixelFormat &pixelFormat);

private:
	uint32_t fourcc_;
};

std::ostream &operator<<(std::ostream &out, const V4L2PixelFormat &f);

} /* namespace libcamera */

namespace std {

template<>
struct hash<libcamera::V4L2PixelFormat> {
	size_t operator()(libcamera::V4L2PixelFormat const &format) const noexcept
	{
		return format.fourcc();
	}
};

} /* namespace std */
