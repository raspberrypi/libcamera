/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Pixel Format
 */

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"

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
 * (see linux/drm_fourcc.h). Constant expressions for all pixel formats
 * supported by libcamera are available in libcamera/formats.h.
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
	const PixelFormatInfo &info = PixelFormatInfo::info(*this);

	if (!info.isValid()) {
		if (*this == PixelFormat())
			return "<INVALID>";

		char fourcc[7] = { '<',
				   static_cast<char>(fourcc_),
				   static_cast<char>(fourcc_ >> 8),
				   static_cast<char>(fourcc_ >> 16),
				   static_cast<char>(fourcc_ >> 24),
				   '>' };

		for (unsigned int i = 1; i < 5; i++) {
			if (!isprint(fourcc[i]))
				fourcc[i] = '.';
		}

		return fourcc;
	}

	return info.name;
}

/**
 * \brief Create a PixelFormat from a string
 * \return The PixelFormat represented by the \a name if known, or an
 * invalid pixel format otherwise.
 */
PixelFormat PixelFormat::fromString(const std::string &name)
{
	return PixelFormatInfo::info(name).format;
}

/**
 * \brief Insert a text representation of a PixelFormat into an output stream
 * \param[in] out The output stream
 * \param[in] f The PixelFormat
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const PixelFormat &f)
{
	out << f.toString();
	return out;
}

} /* namespace libcamera */
