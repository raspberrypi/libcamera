/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * formats.cpp - libcamera image formats
 */

#include "formats.h"

#include <errno.h>

#include "log.h"

/**
 * \file formats.h
 * \brief Types and helper methods to handle libcamera image formats
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Formats)

/**
 * \class ImageFormats
 * \brief Describe V4L2Device and V4L2SubDevice image formats
 *
 * This class stores a list of image formats, each associated with a
 * corresponding set of image sizes. It is used to describe the formats and
 * sizes supported by a V4L2Device or V4L2Subdevice.
 *
 * Formats are stored as an integer. When used for a V4L2Device, the image
 * formats are fourcc pixel formats. When used for a V4L2Subdevice they are
 * media bus codes. Both are defined by the V4L2 specification.
 *
 * Sizes are stored as a list of SizeRange.
 */

/**
 * \brief Add a format and corresponding sizes to the description
 * \param[in] format Pixel format or media bus code to describe
 * \param[in] sizes List of supported size ranges for the format
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EEXIST The format is already described
 */
int ImageFormats::addFormat(unsigned int format, const std::vector<SizeRange> &sizes)
{
	if (data_.find(format) != data_.end())
		return -EEXIST;

	data_[format] = sizes;

	return 0;
}

/**
 * \brief Check if the list of devices supported formats is empty
 * \return True if the list of supported formats is empty
 */
bool ImageFormats::isEmpty() const
{
	return data_.empty();
}

/**
 * \brief Retrieve a list of all supported image formats
 * \return List of pixel formats or media bus codes
 */
std::vector<unsigned int> ImageFormats::formats() const
{
	std::vector<unsigned int> formats;
	formats.reserve(data_.size());

	/* \todo: Should this be cached instead of computed each time? */
	for (auto const &it : data_)
		formats.push_back(it.first);

	return formats;
}

/**
 * \brief Retrieve all sizes for a specific format
 * \param[in] format The pixel format or mbus code
 *
 * Retrieve all size ranges for a specific format. For V4L2Device \a format is a
 * pixel format while for a V4L2Subdevice \a format is a media bus code.
 *
 * \return The list of image sizes supported for \a format, or an empty list if
 * the format is not supported
 */
const std::vector<SizeRange> &ImageFormats::sizes(unsigned int format) const
{
	static const std::vector<SizeRange> empty;

	auto const &it = data_.find(format);
	if (it == data_.end())
		return empty;

	return it->second;
}

/**
 * \brief Retrieve the map that associates formats to image sizes
 * \return The map that associates formats to image sizes
 */
const std::map<unsigned int, std::vector<SizeRange>> &ImageFormats::data() const
{
	return data_;
}

/**
 * \class PixelFormatInfo
 * \brief Information about pixel formats
 *
 * The PixelFormatInfo class groups together information describing a pixel
 * format. It facilitates handling of pixel formats by providing data commonly
 * used in pipeline handlers.
 *
 * \var PixelFormatInfo::format
 * \brief The PixelFormat described by this instance
 *
 * \var PixelFormatInfo::v4l2Format
 * \brief The V4L2 pixel format corresponding to the PixelFormat
 */

namespace {

const std::map<PixelFormat, PixelFormatInfo> pixelFormatInfo{
	/* RGB formats. */
	{ PixelFormat(DRM_FORMAT_BGR888), {
		.format = PixelFormat(DRM_FORMAT_BGR888),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_RGB24),
	} },
	{ PixelFormat(DRM_FORMAT_RGB888), {
		.format = PixelFormat(DRM_FORMAT_RGB888),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_BGR24),
	} },
	{ PixelFormat(DRM_FORMAT_ABGR8888), {
		.format = PixelFormat(DRM_FORMAT_ABGR8888),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_RGBA32),
	} },
	{ PixelFormat(DRM_FORMAT_ARGB8888), {
		.format = PixelFormat(DRM_FORMAT_ARGB8888),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_ABGR32),
	} },
	{ PixelFormat(DRM_FORMAT_BGRA8888), {
		.format = PixelFormat(DRM_FORMAT_BGRA8888),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_ARGB32),
	} },
	{ PixelFormat(DRM_FORMAT_RGBA8888), {
		.format = PixelFormat(DRM_FORMAT_RGBA8888),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_BGRA32),
	} },

	/* YUV packed formats. */
	{ PixelFormat(DRM_FORMAT_YUYV), {
		.format = PixelFormat(DRM_FORMAT_YUYV),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_YUYV),
	} },
	{ PixelFormat(DRM_FORMAT_YVYU), {
		.format = PixelFormat(DRM_FORMAT_YVYU),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_YVYU),
	} },
	{ PixelFormat(DRM_FORMAT_UYVY), {
		.format = PixelFormat(DRM_FORMAT_UYVY),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_UYVY),
	} },
	{ PixelFormat(DRM_FORMAT_VYUY), {
		.format = PixelFormat(DRM_FORMAT_VYUY),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_VYUY),
	} },

	/* YUV planar formats. */
	{ PixelFormat(DRM_FORMAT_NV16), {
		.format = PixelFormat(DRM_FORMAT_NV16),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_NV16),
	} },
	{ PixelFormat(DRM_FORMAT_NV61), {
		.format = PixelFormat(DRM_FORMAT_NV61),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_NV61),
	} },
	{ PixelFormat(DRM_FORMAT_NV12), {
		.format = PixelFormat(DRM_FORMAT_NV12),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_NV12),
	} },
	{ PixelFormat(DRM_FORMAT_NV21), {
		.format = PixelFormat(DRM_FORMAT_NV21),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_NV21),
	} },

	/* Greyscale formats. */
	{ PixelFormat(DRM_FORMAT_R8), {
		.format = PixelFormat(DRM_FORMAT_R8),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_GREY),
	} },

	/* Compressed formats. */
	{ PixelFormat(DRM_FORMAT_MJPEG), {
		.format = PixelFormat(DRM_FORMAT_MJPEG),
		.v4l2Format = V4L2PixelFormat(V4L2_PIX_FMT_MJPEG),
	} },
};

} /* namespace */

/**
 * \fn bool PixelFormatInfo::isValid() const
 * \brief Check if the pixel format info is valid
 * \return True if the pixel format info is valid, false otherwise
 */

/**
 * \brief Retrieve information about a pixel format
 * \param[in] format The pixel format
 * \return The PixelFormatInfo describing the \a format if known, or an invalid
 * PixelFormatInfo otherwise
 */
const PixelFormatInfo &PixelFormatInfo::info(const PixelFormat &format)
{
	static const PixelFormatInfo invalid{};

	const auto iter = pixelFormatInfo.find(format);
	if (iter == pixelFormatInfo.end()) {
		LOG(Formats, Warning)
			<< "Unsupported pixel format "
			<< format.toString();
		return invalid;
	}

	return iter->second;
}

} /* namespace libcamera */
