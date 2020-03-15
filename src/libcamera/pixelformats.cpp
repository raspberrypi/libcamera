/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * pixelformats.cpp - libcamera pixel formats
 */

#include <libcamera/pixelformats.h>

/**
 * \file pixelformats.h
 * \brief libcamera pixel formats
 */

namespace libcamera {

/**
 * \class PixelFormat
 * \brief libcamera image pixel format
 *
 * The PixelFormat type describes the format of images in the public libcamera
 * API. It stores a FourCC value as a 32-bit unsigned integer and a set of
 * modifiers. The FourCC and modifiers values are defined in the Linux kernel
 * DRM/KMS API (see linux/drm_fourcc.h).
 */

/**
 * \brief Construct a PixelFormat with an invalid format
 *
 * PixelFormat instances constructed with the default constructor are
 * invalid, calling the isValid() function returns false.
 */
PixelFormat::PixelFormat()
	: fourcc_(0)
{
}

/**
 * \brief Construct a PixelFormat from a DRM FourCC and a set of modifiers
 * \param[in] fourcc A DRM FourCC
 * \param[in] modifiers A set of DRM FourCC modifiers
 */
PixelFormat::PixelFormat(uint32_t fourcc, const std::set<uint64_t> &modifiers)
	: fourcc_(fourcc), modifiers_(modifiers)
{
}

/**
 * \brief Compare pixel formats for equality
 * \return True if the two pixel formats are equal, false otherwise
 */
bool PixelFormat::operator==(const PixelFormat &other) const
{
	return fourcc_ == other.fourcc() && modifiers_ == other.modifiers_;
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
	return modifiers_ < modifiers_;
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
 * \fn PixelFormat::modifiers() const
 * \brief Retrieve the pixel format modifiers
 * \return Set of DRM modifiers
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
