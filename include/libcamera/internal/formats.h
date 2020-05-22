/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * formats.h - libcamera image formats
 */

#ifndef __LIBCAMERA_FORMATS_H__
#define __LIBCAMERA_FORMATS_H__

#include <map>
#include <vector>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/v4l2_pixelformat.h"

namespace libcamera {

class ImageFormats
{
public:
	int addFormat(unsigned int format, const std::vector<SizeRange> &sizes);

	bool isEmpty() const;
	std::vector<unsigned int> formats() const;
	const std::vector<SizeRange> &sizes(unsigned int format) const;
	const std::map<unsigned int, std::vector<SizeRange>> &data() const;

private:
	std::map<unsigned int, std::vector<SizeRange>> data_;
};

class PixelFormatInfo
{
public:
	enum ColourEncoding {
		ColourEncodingRGB,
		ColourEncodingYUV,
		ColourEncodingRAW,
	};

	bool isValid() const { return format.isValid(); }

	static const PixelFormatInfo &info(const PixelFormat &format);

	/* \todo Add support for non-contiguous memory planes */
	PixelFormat format;
	V4L2PixelFormat v4l2Format;
	unsigned int bitsPerPixel;
	enum ColourEncoding colourEncoding;
	bool packed;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_FORMATS_H__ */
