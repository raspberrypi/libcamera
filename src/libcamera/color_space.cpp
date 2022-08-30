/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * color_space.cpp - color spaces.
 */

#include <libcamera/color_space.h>

#include <algorithm>
#include <array>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

#include <libcamera/base/utils.h>

#include "libcamera/internal/formats.h"

/**
 * \file color_space.h
 * \brief Class and enums to represent color spaces
 */

namespace libcamera {

/**
 * \class ColorSpace
 * \brief Class to describe a color space
 *
 * The ColorSpace class defines the color primaries, the transfer function,
 * the Y'CbCr encoding associated with the color space, and the range
 * (sometimes also referred to as the quantisation) of the color space.
 *
 * Certain combinations of these fields form well-known standard color
 * spaces such as "sRGB" or "Rec709".
 *
 * In the strictest sense a "color space" formally only refers to the
 * color primaries and white point. Here, however, the ColorSpace class
 * adopts the common broader usage that includes the transfer function,
 * Y'CbCr encoding method and quantisation.
 *
 * More information on color spaces is available in the V4L2 documentation, see
 * in particular
 *
 * - <a href="https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html#col-srgb">sRGB</a>
 * - <a href="https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html#col-jpeg">JPEG</a>
 * - <a href="https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html#col-smpte-170m">SMPTE 170M</a>
 * - <a href="https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html#col-rec709">Rec.709</a>
 * - <a href="https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/colorspaces-details.html#col-bt2020">Rec.2020</a>
 *
 * Note that there is no guarantee of a 1:1 mapping between color space names
 * and definitions in libcamera and V4L2. Two notable differences are
 *
 * - The sRGB libcamera color space is defined for RGB formats only with no
 *   Y'CbCr encoding and a full quantization range, while the V4L2 SRGB color
 *   space has a Y'CbCr encoding and a limited quantization range.
 * - The sYCC libcamera color space is called JPEG in V4L2 due to historical
 *   reasons.
 *
 * \todo Define the color space fully in the libcamera API to avoid referencing
 * V4L2
 */

/**
 * \enum ColorSpace::Primaries
 * \brief The color primaries for this color space
 *
 * \var ColorSpace::Primaries::Raw
 * \brief These are raw colors directly from a sensor, the primaries are
 * unspecified
 *
 * \var ColorSpace::Primaries::Smpte170m
 * \brief SMPTE 170M color primaries
 *
 * \var ColorSpace::Primaries::Rec709
 * \brief Rec.709 color primaries
 *
 * \var ColorSpace::Primaries::Rec2020
 * \brief Rec.2020 color primaries
 */

/**
 * \enum ColorSpace::TransferFunction
 * \brief The transfer function used for this color space
 *
 * \var ColorSpace::TransferFunction::Linear
 * \brief This color space uses a linear (identity) transfer function
 *
 * \var ColorSpace::TransferFunction::Srgb
 * \brief sRGB transfer function
 *
 * \var ColorSpace::TransferFunction::Rec709
 * \brief Rec.709 transfer function
 */

/**
 * \enum ColorSpace::YcbcrEncoding
 * \brief The Y'CbCr encoding
 *
 * \var ColorSpace::YcbcrEncoding::None
 * \brief There is no defined Y'CbCr encoding (used for non-YUV formats)
 *
 * \var ColorSpace::YcbcrEncoding::Rec601
 * \brief Rec.601 Y'CbCr encoding
 *
 * \var ColorSpace::YcbcrEncoding::Rec709
 * \brief Rec.709 Y'CbCr encoding
 *
 * \var ColorSpace::YcbcrEncoding::Rec2020
 * \brief Rec.2020 Y'CbCr encoding
 */

/**
 * \enum ColorSpace::Range
 * \brief The range (sometimes "quantisation") for this color space
 *
 * \var ColorSpace::Range::Full
 * \brief This color space uses full range pixel values
 *
 * \var ColorSpace::Range::Limited
 * \brief This color space uses limited range pixel values, being
 * 16 to 235 for Y' and 16 to 240 for Cb and Cr (8 bits per sample)
 * or 64 to 940 for Y' and 16 to 960 for Cb and Cr (10 bits)
 */

/**
 * \fn ColorSpace::ColorSpace(Primaries p, TransferFunction t, Encoding e, Range r)
 * \brief Construct a ColorSpace from explicit values
 * \param[in] p The color primaries
 * \param[in] t The transfer function for the color space
 * \param[in] e The Y'CbCr encoding
 * \param[in] r The range of the pixel values in this color space
 */

/**
 * \brief A constant representing a raw color space (from a sensor)
 */
const ColorSpace ColorSpace::Raw = {
	Primaries::Raw,
	TransferFunction::Linear,
	YcbcrEncoding::None,
	Range::Full
};

/**
 * \brief A constant representing the sRGB color space (RGB formats only)
 */
const ColorSpace ColorSpace::Srgb = {
	Primaries::Rec709,
	TransferFunction::Srgb,
	YcbcrEncoding::None,
	Range::Full
};

/**
 * \brief A constant representing the sYCC color space, typically used for
 * encoding JPEG images
 */
const ColorSpace ColorSpace::Sycc = {
	Primaries::Rec709,
	TransferFunction::Srgb,
	YcbcrEncoding::Rec601,
	Range::Full
};

/**
 * \brief A constant representing the SMPTE170M color space
 */
const ColorSpace ColorSpace::Smpte170m = {
	Primaries::Smpte170m,
	TransferFunction::Rec709,
	YcbcrEncoding::Rec601,
	Range::Limited
};

/**
 * \brief A constant representing the Rec.709 color space
 */
const ColorSpace ColorSpace::Rec709 = {
	Primaries::Rec709,
	TransferFunction::Rec709,
	YcbcrEncoding::Rec709,
	Range::Limited
};

/**
 * \brief A constant representing the Rec.2020 color space
 */
const ColorSpace ColorSpace::Rec2020 = {
	Primaries::Rec2020,
	TransferFunction::Rec709,
	YcbcrEncoding::Rec2020,
	Range::Limited
};

/**
 * \var ColorSpace::primaries
 * \brief The color primaries of this color space
 */

/**
 * \var ColorSpace::transferFunction
 * \brief The transfer function used by this color space
 */

/**
 * \var ColorSpace::ycbcrEncoding
 * \brief The Y'CbCr encoding used by this color space
 */

/**
 * \var ColorSpace::range
 * \brief The pixel range used with by color space
 */

namespace {

const std::array<std::pair<ColorSpace, const char *>, 6> colorSpaceNames = { {
	{ ColorSpace::Raw, "RAW" },
	{ ColorSpace::Srgb, "sRGB" },
	{ ColorSpace::Sycc, "sYCC" },
	{ ColorSpace::Smpte170m, "SMPTE170M" },
	{ ColorSpace::Rec709, "Rec709" },
	{ ColorSpace::Rec2020, "Rec2020" },
} };

const std::map<ColorSpace::Primaries, std::string> primariesNames = {
	{ ColorSpace::Primaries::Raw, "RAW" },
	{ ColorSpace::Primaries::Smpte170m, "SMPTE170M" },
	{ ColorSpace::Primaries::Rec709, "Rec709" },
	{ ColorSpace::Primaries::Rec2020, "Rec2020" },
};

const std::map<ColorSpace::TransferFunction, std::string> transferNames = {
	{ ColorSpace::TransferFunction::Linear, "Linear" },
	{ ColorSpace::TransferFunction::Srgb, "sRGB" },
	{ ColorSpace::TransferFunction::Rec709, "Rec709" },
};

const std::map<ColorSpace::YcbcrEncoding, std::string> encodingNames = {
	{ ColorSpace::YcbcrEncoding::None, "None" },
	{ ColorSpace::YcbcrEncoding::Rec601, "Rec601" },
	{ ColorSpace::YcbcrEncoding::Rec709, "Rec709" },
	{ ColorSpace::YcbcrEncoding::Rec2020, "Rec2020" },
};

const std::map<ColorSpace::Range, std::string> rangeNames = {
	{ ColorSpace::Range::Full, "Full" },
	{ ColorSpace::Range::Limited, "Limited" },
};

} /* namespace */

/**
 * \brief Assemble and return a readable string representation of the
 * ColorSpace
 *
 * If the color space matches a standard ColorSpace (such as ColorSpace::Sycc)
 * then the short name of the color space ("sYCC") is returned. Otherwise
 * the four constituent parts of the ColorSpace are assembled into a longer
 * string.
 *
 * \return A string describing the ColorSpace
 */
std::string ColorSpace::toString() const
{
	/* Print out a brief name only for standard color spaces. */

	auto it = std::find_if(colorSpaceNames.begin(), colorSpaceNames.end(),
			       [this](const auto &item) {
				       return *this == item.first;
			       });
	if (it != colorSpaceNames.end())
		return std::string(it->second);

	/* Assemble a name made of the constituent fields. */

	auto itPrimaries = primariesNames.find(primaries);
	std::string primariesName =
		itPrimaries == primariesNames.end() ? "Invalid" : itPrimaries->second;

	auto itTransfer = transferNames.find(transferFunction);
	std::string transferName =
		itTransfer == transferNames.end() ? "Invalid" : itTransfer->second;

	auto itEncoding = encodingNames.find(ycbcrEncoding);
	std::string encodingName =
		itEncoding == encodingNames.end() ? "Invalid" : itEncoding->second;

	auto itRange = rangeNames.find(range);
	std::string rangeName =
		itRange == rangeNames.end() ? "Invalid" : itRange->second;

	std::stringstream ss;
	ss << primariesName << "/" << transferName << "/" << encodingName << "/" << rangeName;

	return ss.str();
}

/**
 * \brief Assemble and return a readable string representation of an
 * optional ColorSpace
 *
 * This is a convenience helper to easily obtain a string representation
 * for a ColorSpace in parts of the libcamera API where it is stored in a
 * std::optional<>. If the ColorSpace is set, this function returns
 * \a colorSpace.toString(), otherwise it returns "Unset".
 *
 * \return A string describing the optional ColorSpace
 */
std::string ColorSpace::toString(const std::optional<ColorSpace> &colorSpace)
{
	if (!colorSpace)
		return "Unset";

	return colorSpace->toString();
}

/**
 * \brief Construct a color space from a string
 * \param[in] str The string
 *
 * The string \a str can contain the name of a well-known color space, or be
 * made of the four color space components separated by a '/' character, ordered
 * as
 *
 * \verbatim primaries '/' transferFunction '/' ycbcrEncoding '/' range \endverbatim
 *
 * Any failure to parse the string, either because it doesn't match the expected
 * format, or because the one of the names isn't recognized, will cause this
 * function to return std::nullopt.
 *
 * \return The ColorSpace corresponding to the string, or std::nullopt if the
 * string doesn't describe a known color space
 */
std::optional<ColorSpace> ColorSpace::fromString(const std::string &str)
{
	/* First search for a standard color space name match. */
	auto itColorSpace = std::find_if(colorSpaceNames.begin(), colorSpaceNames.end(),
					 [&str](const auto &item) {
						 return str == item.second;
					 });
	if (itColorSpace != colorSpaceNames.end())
		return itColorSpace->first;

	/*
	 * If not found, the string must contain the four color space
	 * components separated by a '/' character.
	 */
	const auto &split = utils::split(str, "/");
	std::vector<std::string> components{ split.begin(), split.end() };

	if (components.size() != 4)
		return std::nullopt;

	ColorSpace colorSpace = ColorSpace::Raw;

	/* Color primaries */
	auto itPrimaries = std::find_if(primariesNames.begin(), primariesNames.end(),
					[&components](const auto &item) {
						return components[0] == item.second;
					});
	if (itPrimaries == primariesNames.end())
		return std::nullopt;

	colorSpace.primaries = itPrimaries->first;

	/* Transfer function */
	auto itTransfer = std::find_if(transferNames.begin(), transferNames.end(),
				       [&components](const auto &item) {
					       return components[1] == item.second;
				       });
	if (itTransfer == transferNames.end())
		return std::nullopt;

	colorSpace.transferFunction = itTransfer->first;

	/* YCbCr encoding */
	auto itEncoding = std::find_if(encodingNames.begin(), encodingNames.end(),
				       [&components](const auto &item) {
					       return components[2] == item.second;
				       });
	if (itEncoding == encodingNames.end())
		return std::nullopt;

	colorSpace.ycbcrEncoding = itEncoding->first;

	/* Quantization range */
	auto itRange = std::find_if(rangeNames.begin(), rangeNames.end(),
				    [&components](const auto &item) {
					    return components[3] == item.second;
				    });
	if (itRange == rangeNames.end())
		return std::nullopt;

	colorSpace.range = itRange->first;

	return colorSpace;
}

/**
 * \brief Adjust the color space to match a pixel format
 * \param[in] format The pixel format
 *
 * Not all combinations of pixel formats and color spaces make sense. For
 * instance, nobody uses a limited quantization range with raw Bayer formats,
 * and the YcbcrEncoding::None encoding isn't valid for YUV formats. This
 * function adjusts the ColorSpace to make it compatible with the given \a
 * format, by applying the following rules:
 *
 * - The color space for RAW formats must be Raw.
 * - The Y'CbCr encoding and quantization range for RGB formats must be
 *   YcbcrEncoding::None and Range::Full respectively.
 * - The Y'CbCr encoding for YUV formats must not be YcbcrEncoding::None. The
 *   best encoding is in that case guessed based on the primaries and transfer
 *   function.
 *
 * \return True if the color space has been adjusted, or false if it was
 * already compatible with the format and hasn't been changed
 */
bool ColorSpace::adjust(PixelFormat format)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(format);
	bool adjusted = false;

	switch (info.colourEncoding) {
	case PixelFormatInfo::ColourEncodingRAW:
		/* Raw formats must use the raw color space. */
		if (*this != ColorSpace::Raw) {
			*this = ColorSpace::Raw;
			adjusted = true;
		}
		break;

	case PixelFormatInfo::ColourEncodingRGB:
		/*
		 * RGB formats can't have a Y'CbCr encoding, and must use full
		 * range quantization.
		 */
		if (ycbcrEncoding != YcbcrEncoding::None) {
			ycbcrEncoding = YcbcrEncoding::None;
			adjusted = true;
		}

		if (range != Range::Full) {
			range = Range::Full;
			adjusted = true;
		}
		break;

	case PixelFormatInfo::ColourEncodingYUV:
		if (ycbcrEncoding != YcbcrEncoding::None)
			break;

		/*
		 * YUV formats must have a Y'CbCr encoding. Infer the most
		 * probable option from the transfer function and primaries.
		 */
		switch (transferFunction) {
		case TransferFunction::Linear:
			/*
			 * Linear YUV is not used in any standard color space,
			 * pick the widely supported and used Rec601 as default.
			 */
			ycbcrEncoding = YcbcrEncoding::Rec601;
			break;

		case TransferFunction::Rec709:
			switch (primaries) {
			/* Raw should never happen. */
			case Primaries::Raw:
			case Primaries::Smpte170m:
				ycbcrEncoding = YcbcrEncoding::Rec601;
				break;
			case Primaries::Rec709:
				ycbcrEncoding = YcbcrEncoding::Rec709;
				break;
			case Primaries::Rec2020:
				ycbcrEncoding = YcbcrEncoding::Rec2020;
				break;
			}
			break;

		case TransferFunction::Srgb:
			/*
			 * Only the sYCC color space uses the sRGB transfer
			 * function, the corresponding encoding is Rec601.
			 */
			ycbcrEncoding = YcbcrEncoding::Rec601;
			break;
		}

		adjusted = true;
		break;
	}

	return adjusted;
}

/**
 * \brief Compare color spaces for equality
 * \return True if the two color spaces are identical, false otherwise
 */
bool operator==(const ColorSpace &lhs, const ColorSpace &rhs)
{
	return lhs.primaries == rhs.primaries &&
	       lhs.transferFunction == rhs.transferFunction &&
	       lhs.ycbcrEncoding == rhs.ycbcrEncoding &&
	       lhs.range == rhs.range;
}

/**
 * \fn bool operator!=(const ColorSpace &lhs, const ColorSpace &rhs)
 * \brief Compare color spaces for inequality
 * \return True if the two color spaces are not identical, false otherwise
 */

} /* namespace libcamera */
