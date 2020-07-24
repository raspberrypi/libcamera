/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * v4l2_pixelformat.cpp - V4L2 Pixel Format
 */

#include "libcamera/internal/v4l2_pixelformat.h"

#include <ctype.h>
#include <map>
#include <string.h>

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/log.h"

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

namespace {

const std::map<V4L2PixelFormat, PixelFormat> vpf2pf{
	/* RGB formats. */
	{ V4L2PixelFormat(V4L2_PIX_FMT_RGB565), formats::RGB565 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_RGB24), formats::BGR888 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_BGR24), formats::RGB888 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_RGBA32), formats::ABGR8888 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_ABGR32), formats::ARGB8888 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_ARGB32), formats::BGRA8888 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_BGRA32), formats::RGBA8888 },

	/* YUV packed formats. */
	{ V4L2PixelFormat(V4L2_PIX_FMT_YUYV), formats::YUYV },
	{ V4L2PixelFormat(V4L2_PIX_FMT_YVYU), formats::YVYU },
	{ V4L2PixelFormat(V4L2_PIX_FMT_UYVY), formats::UYVY },
	{ V4L2PixelFormat(V4L2_PIX_FMT_VYUY), formats::VYUY },

	/* YUV planar formats. */
	{ V4L2PixelFormat(V4L2_PIX_FMT_NV16), formats::NV16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_NV61), formats::NV61 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_NV12), formats::NV12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_NV21), formats::NV21 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_YUV420), formats::YUV420 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_YVU420), formats::YVU420 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_YUV422P), formats::YUV422 },

	/* Greyscale formats. */
	{ V4L2PixelFormat(V4L2_PIX_FMT_GREY), formats::R8 },

	/* Bayer formats. */
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8), formats::SBGGR8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8), formats::SGBRG8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8), formats::SGRBG8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8), formats::SRGGB8 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10), formats::SBGGR10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10), formats::SGBRG10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10), formats::SGRBG10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10), formats::SRGGB10 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10P), formats::SBGGR10_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10P), formats::SGBRG10_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10P), formats::SGRBG10_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10P), formats::SRGGB10_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12), formats::SBGGR12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12), formats::SGBRG12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12), formats::SGRBG12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12), formats::SRGGB12 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12P), formats::SBGGR12_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12P), formats::SGBRG12_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12P), formats::SGRBG12_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12P), formats::SRGGB12_CSI2P },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16), formats::SBGGR16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16), formats::SGBRG16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16), formats::SGRBG16 },
	{ V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16), formats::SRGGB16 },

	/* Compressed formats. */
	{ V4L2PixelFormat(V4L2_PIX_FMT_MJPEG), formats::MJPEG },
};

} /* namespace */

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

/**
 * \brief Convert the V4L2 pixel format to the corresponding PixelFormat
 * \return The PixelFormat corresponding to the V4L2 pixel format
 */
PixelFormat V4L2PixelFormat::toPixelFormat() const
{
	const auto iter = vpf2pf.find(*this);
	if (iter == vpf2pf.end()) {
		LOG(V4L2, Warning)
			<< "Unsupported V4L2 pixel format "
			<< toString();
		return PixelFormat();
	}

	return iter->second;
}

/**
 * \brief Convert \a pixelFormat to its corresponding V4L2PixelFormat
 * \param[in] pixelFormat The PixelFormat to convert
 * \param[in] multiplanar V4L2 Multiplanar API support flag
 *
 * Multiple V4L2 formats may exist for one PixelFormat when the format uses
 * multiple planes, as V4L2 defines separate 4CCs for contiguous and separate
 * planes formats. Set the \a multiplanar parameter to false to select a format
 * with contiguous planes, or to true to select a format with non-contiguous
 * planes.
 *
 * \return The V4L2PixelFormat corresponding to \a pixelFormat
 */
V4L2PixelFormat V4L2PixelFormat::fromPixelFormat(const PixelFormat &pixelFormat,
						 bool multiplanar)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(pixelFormat);
	if (!info.isValid())
		return V4L2PixelFormat();

	return info.v4l2Format;
}

} /* namespace libcamera */
