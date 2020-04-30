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

/**
 * \brief Convert the V4L2 pixel format to the corresponding PixelFormat
 * \return The PixelFormat corresponding to the V4L2 pixel format
 */
PixelFormat V4L2PixelFormat::toPixelFormat() const
{
	switch (fourcc_) {
	/* RGB formats. */
	case V4L2_PIX_FMT_RGB24:
		return PixelFormat(DRM_FORMAT_BGR888);
	case V4L2_PIX_FMT_BGR24:
		return PixelFormat(DRM_FORMAT_RGB888);
	case V4L2_PIX_FMT_RGBA32:
		return PixelFormat(DRM_FORMAT_ABGR8888);
	case V4L2_PIX_FMT_ABGR32:
		return PixelFormat(DRM_FORMAT_ARGB8888);
	case V4L2_PIX_FMT_ARGB32:
		return PixelFormat(DRM_FORMAT_BGRA8888);
	case V4L2_PIX_FMT_BGRA32:
		return PixelFormat(DRM_FORMAT_RGBA8888);

	/* YUV packed formats. */
	case V4L2_PIX_FMT_YUYV:
		return PixelFormat(DRM_FORMAT_YUYV);
	case V4L2_PIX_FMT_YVYU:
		return PixelFormat(DRM_FORMAT_YVYU);
	case V4L2_PIX_FMT_UYVY:
		return PixelFormat(DRM_FORMAT_UYVY);
	case V4L2_PIX_FMT_VYUY:
		return PixelFormat(DRM_FORMAT_VYUY);

	/* YUY planar formats. */
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV16M:
		return PixelFormat(DRM_FORMAT_NV16);
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV61M:
		return PixelFormat(DRM_FORMAT_NV61);
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
		return PixelFormat(DRM_FORMAT_NV12);
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		return PixelFormat(DRM_FORMAT_NV21);

	/* Greyscale formats. */
	case V4L2_PIX_FMT_GREY:
		return PixelFormat(DRM_FORMAT_R8);

	/* Compressed formats. */
	case V4L2_PIX_FMT_MJPEG:
		return PixelFormat(DRM_FORMAT_MJPEG);

	/* V4L2 formats not yet supported by DRM. */
	default:
		LOG(V4L2, Warning)
			<< "Unsupported V4L2 pixel format "
			<< toString();
		return PixelFormat();
	}
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
	switch (pixelFormat) {
	/* RGB formats. */
	case DRM_FORMAT_BGR888:
		return V4L2PixelFormat(V4L2_PIX_FMT_RGB24);
	case DRM_FORMAT_RGB888:
		return V4L2PixelFormat(V4L2_PIX_FMT_BGR24);
	case DRM_FORMAT_ABGR8888:
		return V4L2PixelFormat(V4L2_PIX_FMT_RGBA32);
	case DRM_FORMAT_ARGB8888:
		return V4L2PixelFormat(V4L2_PIX_FMT_ABGR32);
	case DRM_FORMAT_BGRA8888:
		return V4L2PixelFormat(V4L2_PIX_FMT_ARGB32);
	case DRM_FORMAT_RGBA8888:
		return V4L2PixelFormat(V4L2_PIX_FMT_BGRA32);

	/* YUV packed formats. */
	case DRM_FORMAT_YUYV:
		return V4L2PixelFormat(V4L2_PIX_FMT_YUYV);
	case DRM_FORMAT_YVYU:
		return V4L2PixelFormat(V4L2_PIX_FMT_YVYU);
	case DRM_FORMAT_UYVY:
		return V4L2PixelFormat(V4L2_PIX_FMT_UYVY);
	case DRM_FORMAT_VYUY:
		return V4L2PixelFormat(V4L2_PIX_FMT_VYUY);

	/*
	 * YUY planar formats.
	 * \todo Add support for non-contiguous memory planes
	 * \todo Select the format variant not only based on \a multiplanar but
	 * also take into account the formats supported by the device.
	 */
	case DRM_FORMAT_NV16:
		return V4L2PixelFormat(V4L2_PIX_FMT_NV16);
	case DRM_FORMAT_NV61:
		return V4L2PixelFormat(V4L2_PIX_FMT_NV61);
	case DRM_FORMAT_NV12:
		return V4L2PixelFormat(V4L2_PIX_FMT_NV12);
	case DRM_FORMAT_NV21:
		return V4L2PixelFormat(V4L2_PIX_FMT_NV21);

	/* Greyscale formats. */
	case DRM_FORMAT_R8:
		return V4L2PixelFormat(V4L2_PIX_FMT_GREY);

	/* Compressed formats. */
	case DRM_FORMAT_MJPEG:
		return V4L2PixelFormat(V4L2_PIX_FMT_MJPEG);

	default:
		LOG(V4L2, Warning)
			<< "Unsupported pixel format "
			<< pixelFormat.toString();
		return V4L2PixelFormat();
	}
}

} /* namespace libcamera */
