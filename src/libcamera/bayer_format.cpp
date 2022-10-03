/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * bayer_format.cpp - Class to represent Bayer formats
 */

#include "libcamera/internal/bayer_format.h"

#include <algorithm>
#include <map>
#include <sstream>
#include <unordered_map>

#include <linux/media-bus-format.h>

#include <libcamera/formats.h>
#include <libcamera/transform.h>

/**
 * \file bayer_format.h
 * \brief Class to represent Bayer formats and manipulate them
 */

namespace libcamera {

/**
 * \class BayerFormat
 * \brief Class to represent a raw image Bayer format
 *
 * This class encodes the different Bayer formats in such a way that they can
 * be easily manipulated. For example, the bit depth or Bayer order can be
 * easily altered - the Bayer order can even be "transformed" in the same
 * manner as happens in many sensors when their horizontal or vertical "flip"
 * controls are set.
 */

/**
 * \enum BayerFormat::Order
 * \brief The order of the colour channels in the Bayer pattern
 *
 * \var BayerFormat::BGGR
 * \brief B then G on the first row, G then R on the second row.
 * \var BayerFormat::GBRG
 * \brief G then B on the first row, R then G on the second row.
 * \var BayerFormat::GRBG
 * \brief G then R on the first row, B then G on the second row.
 * \var BayerFormat::RGGB
 * \brief R then G on the first row, G then B on the second row.
 * \var BayerFormat::MONO
 * \brief Monochrome image data, there is no colour filter array.
 */

/**
 * \enum BayerFormat::Packing
 * \brief Different types of packing that can be applied to a BayerFormat
 *
 * \var BayerFormat::Packing::None
 * \brief No packing
 * \var BayerFormat::Packing::CSI2
 * \brief Format uses MIPI CSI-2 style packing
 * \var BayerFormat::Packing::IPU3
 * \brief Format uses IPU3 style packing
 */

namespace {

/* Define a slightly arbitrary ordering so that we can use a std::map. */
struct BayerFormatComparator {
	constexpr bool operator()(const BayerFormat &lhs, const BayerFormat &rhs) const
	{
		if (lhs.bitDepth < rhs.bitDepth)
			return true;
		else if (lhs.bitDepth > rhs.bitDepth)
			return false;

		if (lhs.order < rhs.order)
			return true;
		else if (lhs.order > rhs.order)
			return false;

		if (lhs.packing < rhs.packing)
			return true;
		else
			return false;
	}
};

struct Formats {
	PixelFormat pixelFormat;
	V4L2PixelFormat v4l2Format;
};

const std::map<BayerFormat, Formats, BayerFormatComparator> bayerToFormat{
	{ { BayerFormat::BGGR, 8, BayerFormat::Packing::None },
		{ formats::SBGGR8, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8) } },
	{ { BayerFormat::GBRG, 8, BayerFormat::Packing::None },
		{ formats::SGBRG8, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8) } },
	{ { BayerFormat::GRBG, 8, BayerFormat::Packing::None },
		{ formats::SGRBG8, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8) } },
	{ { BayerFormat::RGGB, 8, BayerFormat::Packing::None },
		{ formats::SRGGB8, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8) } },
	{ { BayerFormat::BGGR, 10, BayerFormat::Packing::None },
		{ formats::SBGGR10, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10) } },
	{ { BayerFormat::GBRG, 10, BayerFormat::Packing::None },
		{ formats::SGBRG10, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10) } },
	{ { BayerFormat::GRBG, 10, BayerFormat::Packing::None },
		{ formats::SGRBG10, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10) } },
	{ { BayerFormat::RGGB, 10, BayerFormat::Packing::None },
		{ formats::SRGGB10, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10) } },
	{ { BayerFormat::BGGR, 10, BayerFormat::Packing::CSI2 },
		{ formats::SBGGR10_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10P) } },
	{ { BayerFormat::GBRG, 10, BayerFormat::Packing::CSI2 },
		{ formats::SGBRG10_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10P) } },
	{ { BayerFormat::GRBG, 10, BayerFormat::Packing::CSI2 },
		{ formats::SGRBG10_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10P) } },
	{ { BayerFormat::RGGB, 10, BayerFormat::Packing::CSI2 },
		{ formats::SRGGB10_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10P) } },
	{ { BayerFormat::BGGR, 10, BayerFormat::Packing::IPU3 },
		{ formats::SBGGR10_IPU3, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SBGGR10) } },
	{ { BayerFormat::GBRG, 10, BayerFormat::Packing::IPU3 },
		{ formats::SGBRG10_IPU3, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SGBRG10) } },
	{ { BayerFormat::GRBG, 10, BayerFormat::Packing::IPU3 },
		{ formats::SGRBG10_IPU3, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SGRBG10) } },
	{ { BayerFormat::RGGB, 10, BayerFormat::Packing::IPU3 },
		{ formats::SRGGB10_IPU3, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SRGGB10) } },
	{ { BayerFormat::BGGR, 12, BayerFormat::Packing::None },
		{ formats::SBGGR12, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12) } },
	{ { BayerFormat::GBRG, 12, BayerFormat::Packing::None },
		{ formats::SGBRG12, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12) } },
	{ { BayerFormat::GRBG, 12, BayerFormat::Packing::None },
		{ formats::SGRBG12, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12) } },
	{ { BayerFormat::RGGB, 12, BayerFormat::Packing::None },
		{ formats::SRGGB12, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12) } },
	{ { BayerFormat::BGGR, 12, BayerFormat::Packing::CSI2 },
		{ formats::SBGGR12_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12P) } },
	{ { BayerFormat::GBRG, 12, BayerFormat::Packing::CSI2 },
		{ formats::SGBRG12_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12P) } },
	{ { BayerFormat::GRBG, 12, BayerFormat::Packing::CSI2 },
		{ formats::SGRBG12_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12P) } },
	{ { BayerFormat::RGGB, 12, BayerFormat::Packing::CSI2 },
		{ formats::SRGGB12_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12P) } },
	{ { BayerFormat::BGGR, 16, BayerFormat::Packing::None },
		{ formats::SBGGR16, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16) } },
	{ { BayerFormat::GBRG, 16, BayerFormat::Packing::None },
		{ formats::SGBRG16, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16) } },
	{ { BayerFormat::GRBG, 16, BayerFormat::Packing::None },
		{ formats::SGRBG16, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16) } },
	{ { BayerFormat::RGGB, 16, BayerFormat::Packing::None },
		{ formats::SRGGB16, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16) } },
	{ { BayerFormat::MONO, 8, BayerFormat::Packing::None },
		{ formats::R8, V4L2PixelFormat(V4L2_PIX_FMT_GREY) } },
	{ { BayerFormat::MONO, 10, BayerFormat::Packing::None },
		{ formats::R10, V4L2PixelFormat(V4L2_PIX_FMT_Y10) } },
	{ { BayerFormat::MONO, 10, BayerFormat::Packing::CSI2 },
		{ formats::R10_CSI2P, V4L2PixelFormat(V4L2_PIX_FMT_Y10P) } },
};

const std::unordered_map<unsigned int, BayerFormat> mbusCodeToBayer{
	{ MEDIA_BUS_FMT_SBGGR8_1X8, { BayerFormat::BGGR, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, { BayerFormat::GBRG, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, { BayerFormat::GRBG, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, { BayerFormat::RGGB, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_ALAW8_1X8, { BayerFormat::BGGR, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG10_ALAW8_1X8, { BayerFormat::GBRG, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG10_ALAW8_1X8, { BayerFormat::GRBG, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB10_ALAW8_1X8, { BayerFormat::RGGB, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8, { BayerFormat::BGGR, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8, { BayerFormat::GBRG, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8, { BayerFormat::GRBG, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8, { BayerFormat::RGGB, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE, { BayerFormat::BGGR, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE, { BayerFormat::BGGR, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE, { BayerFormat::BGGR, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE, { BayerFormat::BGGR, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, { BayerFormat::BGGR, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, { BayerFormat::GBRG, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, { BayerFormat::GRBG, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, { BayerFormat::RGGB, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, { BayerFormat::BGGR, 12, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, { BayerFormat::GBRG, 12, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, { BayerFormat::GRBG, 12, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, { BayerFormat::RGGB, 12, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR14_1X14, { BayerFormat::BGGR, 14, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG14_1X14, { BayerFormat::GBRG, 14, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG14_1X14, { BayerFormat::GRBG, 14, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB14_1X14, { BayerFormat::RGGB, 14, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SBGGR16_1X16, { BayerFormat::BGGR, 16, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGBRG16_1X16, { BayerFormat::GBRG, 16, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SGRBG16_1X16, { BayerFormat::GRBG, 16, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_SRGGB16_1X16, { BayerFormat::RGGB, 16, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_Y8_1X8, { BayerFormat::MONO, 8, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_Y10_1X10, { BayerFormat::MONO, 10, BayerFormat::Packing::None } },
	{ MEDIA_BUS_FMT_Y12_1X12, { BayerFormat::MONO, 12, BayerFormat::Packing::None } },
};

} /* namespace */

/**
 * \fn BayerFormat::BayerFormat()
 * \brief Construct an empty (and invalid) BayerFormat
 */

/**
 * \fn BayerFormat::BayerFormat(Order o, uint8_t b, Packing p)
 * \brief Construct a BayerFormat from explicit values
 * \param[in] o The order of the Bayer pattern
 * \param[in] b The bit depth of the Bayer samples
 * \param[in] p The type of packing applied to the pixel values
 */

/**
 * \brief Retrieve the BayerFormat associated with a media bus code
 * \param[in] mbusCode The media bus code to convert into a BayerFormat
 *
 * The media bus code numeric identifiers are defined by the V4L2 specification.
 */
const BayerFormat &BayerFormat::fromMbusCode(unsigned int mbusCode)
{
	static BayerFormat empty;

	const auto it = mbusCodeToBayer.find(mbusCode);
	if (it == mbusCodeToBayer.end())
		return empty;
	else
		return it->second;
}

/**
 * \fn BayerFormat::isValid()
 * \brief Return whether a BayerFormat is valid
 */

/**
 * \brief Assemble and return a readable string representation of the
 * BayerFormat
 * \return A string describing the BayerFormat
 */
std::string BayerFormat::toString() const
{
	std::stringstream ss;
	ss << *this;

	return ss.str();
}

/**
 * \brief Compare two BayerFormats for equality
 * \return True if order, bitDepth and packing are equal, or false otherwise
 */
bool operator==(const BayerFormat &lhs, const BayerFormat &rhs)
{
	return lhs.order == rhs.order && lhs.bitDepth == rhs.bitDepth &&
	       lhs.packing == rhs.packing;
}

/**
 * \brief Insert a text representation of a BayerFormats into an output stream
 * \param[in] out The output stream
 * \param[in] f The BayerFormat
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const BayerFormat &f)
{
	static const char *orderStrings[] = {
		"BGGR-",
		"GBRG-",
		"GRBG-",
		"RGGB-",
		"MONO-"
	};

	if (!f.isValid() || f.order > BayerFormat::MONO) {
		out << "INVALID";
		return out;
	}

	/* The cast is required to avoid bitDepth being interpreted as a char. */
	out << orderStrings[f.order] << static_cast<unsigned int>(f.bitDepth);

	if (f.packing == BayerFormat::Packing::CSI2)
		out << "-CSI2P";
	else if (f.packing == BayerFormat::Packing::IPU3)
		out << "-IPU3P";

	return out;
}

/**
 * \fn bool operator!=(const BayerFormat &lhs, const BayerFormat &rhs)
 * \brief Compare two BayerFormats for inequality
 * \return True if either order, bitdepth or packing are not equal, or false
 * otherwise
 */

/**
 * \brief Convert a BayerFormat into the corresponding V4L2PixelFormat
 * \return The V4L2PixelFormat corresponding to this BayerFormat
 */
V4L2PixelFormat BayerFormat::toV4L2PixelFormat() const
{
	const auto it = bayerToFormat.find(*this);
	if (it != bayerToFormat.end())
		return it->second.v4l2Format;

	return V4L2PixelFormat();
}

/**
 * \brief Convert \a v4l2Format to the corresponding BayerFormat
 * \param[in] v4l2Format The raw format to convert into a BayerFormat
 * \return The BayerFormat corresponding to \a v4l2Format
 */
BayerFormat BayerFormat::fromV4L2PixelFormat(V4L2PixelFormat v4l2Format)
{
	auto it = std::find_if(bayerToFormat.begin(), bayerToFormat.end(),
			       [v4l2Format](const auto &i) {
				       return i.second.v4l2Format == v4l2Format;
			       });
	if (it != bayerToFormat.end())
		return it->first;

	return BayerFormat();
}

/**
 * \brief Convert a BayerFormat into the corresponding PixelFormat
 * \return The PixelFormat corresponding to this BayerFormat
 */
PixelFormat BayerFormat::toPixelFormat() const
{
	const auto it = bayerToFormat.find(*this);
	if (it != bayerToFormat.end())
		return it->second.pixelFormat;

	return PixelFormat();
}

/**
 * \brief Convert a PixelFormat into the corresponding BayerFormat
 * \return The BayerFormat corresponding to this PixelFormat
 */
BayerFormat BayerFormat::fromPixelFormat(PixelFormat format)
{
	const auto it = std::find_if(bayerToFormat.begin(), bayerToFormat.end(),
				     [format](const auto &i) {
					     return i.second.pixelFormat == format;
				     });
	if (it != bayerToFormat.end())
		return it->first;

	return BayerFormat();
}

/**
 * \brief Apply a transform to this BayerFormat
 * \param[in] t The transform to apply
 *
 * Appplying a transform to an image stored in a Bayer format affects the Bayer
 * order. For example, performing a horizontal flip on the Bayer pattern
 * RGGB causes the RG rows of pixels to become GR, and the GB rows to become BG.
 * The transformed image would have a GRBG order. The bit depth and modifiers
 * are not affected.
 *
 * Horizontal and vertical flips are applied before transpose.
 *
 * \return The transformed Bayer format
 */
BayerFormat BayerFormat::transform(Transform t) const
{
	BayerFormat result = *this;

	if (order == MONO)
		return result;

	/*
	 * Observe that flipping bit 0 of the Order enum performs a horizontal
	 * mirror on the Bayer pattern (e.g. RGGB goes to GRBG). Similarly,
	 * flipping bit 1 performs a vertical mirror operation on it. Hence:
	 */
	if (!!(t & Transform::HFlip))
		result.order = static_cast<Order>(result.order ^ 1);
	if (!!(t & Transform::VFlip))
		result.order = static_cast<Order>(result.order ^ 2);

	if (!!(t & Transform::Transpose) && result.order == 1)
		result.order = static_cast<Order>(2);
	else if (!!(t & Transform::Transpose) && result.order == 2)
		result.order = static_cast<Order>(1);

	return result;
}

/**
 * \var BayerFormat::order
 * \brief The order of the colour channels in the Bayer pattern
 */

/**
 * \var BayerFormat::bitDepth
 * \brief The bit depth of the samples in the Bayer pattern
 */

/**
 * \var BayerFormat::packing
 * \brief Any packing scheme applied to this BayerFormat
 */

} /* namespace libcamera */
