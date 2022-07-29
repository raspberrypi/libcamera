/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * formats.h - libcamera image formats
 */

#pragma once

#include <array>
#include <map>
#include <vector>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/v4l2_pixelformat.h"

namespace libcamera {

class PixelFormatInfo
{
public:
	enum ColourEncoding {
		ColourEncodingRGB,
		ColourEncodingYUV,
		ColourEncodingRAW,
	};

	struct Plane {
		unsigned int bytesPerGroup;
		unsigned int verticalSubSampling;
	};

	bool isValid() const { return format.isValid(); }

	static const PixelFormatInfo &info(const PixelFormat &format);
	static const PixelFormatInfo &info(const V4L2PixelFormat &format);
	static const PixelFormatInfo &info(const std::string &name);

	unsigned int stride(unsigned int width, unsigned int plane,
			    unsigned int align = 1) const;
	unsigned int planeSize(const Size &size, unsigned int plane,
			       unsigned int align = 1) const;
	unsigned int planeSize(unsigned int height, unsigned int plane,
			       unsigned int stride) const;
	unsigned int frameSize(const Size &size, unsigned int align = 1) const;
	unsigned int frameSize(const Size &size,
			       const std::array<unsigned int, 3> &strides) const;

	unsigned int numPlanes() const;

	/* \todo Add support for non-contiguous memory planes */
	const char *name;
	PixelFormat format;
	std::vector<V4L2PixelFormat> v4l2Formats;
	unsigned int bitsPerPixel;
	enum ColourEncoding colourEncoding;
	bool packed;

	unsigned int pixelsPerGroup;

	std::array<Plane, 3> planes;
};

} /* namespace libcamera */
