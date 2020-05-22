/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * v4l2_pixelformat.h - V4L2 Pixel Format
 */
#ifndef __LIBCAMERA_INTERNAL_V4L2_PIXELFORMAT_H__
#define __LIBCAMERA_INTERNAL_V4L2_PIXELFORMAT_H__

#include <stdint.h>
#include <string>

#include <linux/videodev2.h>

#include <libcamera/pixel_format.h>

namespace libcamera {

class V4L2PixelFormat
{
public:
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

	PixelFormat toPixelFormat() const;
	static V4L2PixelFormat fromPixelFormat(const PixelFormat &pixelFormat,
					       bool multiplanar);

private:
	uint32_t fourcc_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_V4L2_PIXELFORMAT_H__ */
