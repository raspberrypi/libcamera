/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * formats.h - libcamera image formats
 */

#ifndef __LIBCAMERA_INTERNAL_FORMATS_H__
#define __LIBCAMERA_INTERNAL_FORMATS_H__

#include <array>
#include <map>
#include <vector>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/v4l2_pixelformat.h"

namespace libcamera {

struct PixelFormatPlaneInfo
{
	unsigned int bytesPerGroup;
	unsigned int verticalSubSampling;
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
	static const PixelFormatInfo &info(const V4L2PixelFormat &format);
	static const PixelFormatInfo &info(const std::string &name);

	unsigned int stride(unsigned int width, unsigned int plane,
			    unsigned int align = 1) const;
	unsigned int frameSize(const Size &size, unsigned int align = 1) const;
	unsigned int frameSize(const Size &size,
			       const std::array<unsigned int, 3> &strides) const;

	unsigned int numPlanes() const;

	/* \todo Add support for non-contiguous memory planes */
	const char *name;
	PixelFormat format;
	V4L2PixelFormat v4l2Format;
	unsigned int bitsPerPixel;
	enum ColourEncoding colourEncoding;
	bool packed;

	unsigned int pixelsPerGroup;

	std::array<PixelFormatPlaneInfo, 3> planes;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_FORMATS_H__ */
