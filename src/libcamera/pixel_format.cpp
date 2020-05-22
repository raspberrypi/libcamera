/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * pixel_format.cpp - libcamera Pixel Format
 */

#include <libcamera/pixel_format.h>

/**
 * \file pixel_format.h
 * \brief libcamera pixel format
 */

namespace libcamera {

/**
 * \class PixelFormat
 * \brief libcamera image pixel format
 *
 * The PixelFormat type describes the format of images in the public libcamera
 * API. It stores a FourCC value as a 32-bit unsigned integer and a modifier.
 * The FourCC and modifier values are defined in the Linux kernel DRM/KMS API
 * (see linux/drm_fourcc.h).
 */

/**
 * \fn PixelFormat::PixelFormat()
 * \brief Construct a PixelFormat with an invalid format
 *
 * PixelFormat instances constructed with the default constructor are
 * invalid, calling the isValid() function returns false.
 */

/**
 * \fn PixelFormat::PixelFormat(uint32_t fourcc, uint64_t modifier)
 * \brief Construct a PixelFormat from a DRM FourCC and a modifier
 * \param[in] fourcc A DRM FourCC
 * \param[in] modifier A DRM FourCC modifier
 */

/**
 * \brief Compare pixel formats for equality
 * \return True if the two pixel formats are equal, false otherwise
 */
bool PixelFormat::operator==(const PixelFormat &other) const
{
	return fourcc_ == other.fourcc() && modifier_ == other.modifier_;
}

/**
 * \fn bool PixelFormat::operator!=(const PixelFormat &other) const
 * \brief Compare pixel formats for inequality
 * \return True if the two pixel formats are not equal, false otherwise
 */

/**
 * \brief Compare pixel formats for smaller than order
 * \return True if \a this is smaller than \a other, false otherwise
 */
bool PixelFormat::operator<(const PixelFormat &other) const
{
	if (fourcc_ < other.fourcc_)
		return true;
	if (fourcc_ > other.fourcc_)
		return false;
	return modifier_ < other.modifier_;
}

/**
 * \fn bool PixelFormat::isValid() const
 * \brief Check if the pixel format is valid
 *
 * PixelFormat instances constructed with the default constructor are
 * invalid. Instances constructed with a FourCC defined in the DRM API
 * are valid. The behaviour is undefined otherwise.
 *
 * \return True if the pixel format is valid, false otherwise
 */

/**
 * \fn PixelFormat::operator uint32_t() const
 * \brief Convert the the pixel format numerical value
 * \return The pixel format numerical value
 */

/**
 * \fn PixelFormat::fourcc() const
 * \brief Retrieve the pixel format FourCC
 * \return DRM FourCC
 */

/**
 * \fn PixelFormat::modifier() const
 * \brief Retrieve the pixel format modifier
 * \return DRM modifier
 */

/**
 * \brief Assemble and return a string describing the pixel format
 * \return A string describing the pixel format
 */
std::string PixelFormat::toString() const
{
	char str[11];
	snprintf(str, 11, "0x%08x", fourcc_);
	return str;
}

} /* namespace libcamera */
