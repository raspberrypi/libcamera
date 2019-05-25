/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * formats.cpp - Libcamera image formats
 */

#include "formats.h"

/**
 * \file formats.h
 * \brief Types and helper methods to handle libcamera image formats
 */

namespace libcamera {

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
	static std::vector<SizeRange> empty;

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

} /* namespace libcamera */
