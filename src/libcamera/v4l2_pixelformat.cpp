/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * v4l2_pixelformat.cpp - V4L2 Pixel Format
 */

#include "v4l2_pixelformat.h"

#include <ctype.h>
#include <map>
#include <string.h>

#include <linux/drm_fourcc.h>

#include <libcamera/pixelformats.h>

#include "log.h"

/**
 * \file v4l2_pixelformat.h
 * \brief V4L2 Pixel Format
 */
namespace libcamera {

LOG_DECLARE_CATEGORY(V4L2)

/**
 * \class V4L2PixelFormat
 * \brief V4L2 pixel format FourCC wrapper
 *
 * The V4L2PixelFormat class describes the pixel format of a V4L2 buffer. It
 * wraps the V4L2 numerical FourCC, and shall be used in all APIs that deal with
 * V4L2 pixel formats. Its purpose is to prevent unintentional confusion of
 * V4L2 and DRM FourCCs in code by catching implicit conversion attempts at
 * compile time.
 *
 * To achieve this goal, construction of a V4L2PixelFormat from an integer value
 * is explicit. To retrieve the integer value of a V4L2PixelFormat, both the
 * explicit value() and implicit uint32_t conversion operators may be used.
 */

/**
 * \fn V4L2PixelFormat::V4L2PixelFormat()
 * \brief Construct a V4L2PixelFormat with an invalid format
 *
 * V4L2PixelFormat instances constructed with the default constructor are
 * invalid, calling the isValid() function returns false.
 */

/**
 * \fn V4L2PixelFormat::V4L2PixelFormat(uint32_t fourcc)
 * \brief Construct a V4L2PixelFormat from a FourCC value
 * \param[in] fourcc The pixel format FourCC numerical value
 */

/**
 * \fn bool V4L2PixelFormat::isValid() const
 * \brief Check if the pixel format is valid
 *
 * V4L2PixelFormat instances constructed with the default constructor are
 * invalid. Instances constructed with a FourCC defined in the V4L2 API are
 * valid. The behaviour is undefined otherwise.
 *
 * \return True if the pixel format is valid, false otherwise
 */

/**
 * \fn uint32_t V4L2PixelFormat::fourcc() const
 * \brief Retrieve the pixel format FourCC numerical value
 * \return The pixel format FourCC numerical value
 */

/**
 * \fn V4L2PixelFormat::operator uint32_t() const
 * \brief Convert to the pixel format FourCC numerical value
 * \return The pixel format FourCC numerical value
 */

/**
 * \brief Assemble and return a string describing the pixel format
 * \return A string describing the pixel format
 */
std::string V4L2PixelFormat::toString() const
{
	if (fourcc_ == 0)
		return "<INVALID>";

	char ss[8] = { static_cast<char>(fourcc_ & 0x7f),
		       static_cast<char>((fourcc_ >> 8) & 0x7f),
		       static_cast<char>((fourcc_ >> 16) & 0x7f),
		       static_cast<char>((fourcc_ >> 24) & 0x7f) };

	for (unsigned int i = 0; i < 4; i++) {
		if (!isprint(ss[i]))
			ss[i] = '.';
	}

	if (fourcc_ & (1 << 31))
		strcat(ss, "-BE");

	return ss;
}

} /* namespace libcamera */
