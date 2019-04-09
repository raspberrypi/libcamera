/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.h - Geometry-related structure
 */

#ifndef __LIBCAMERA_GEOMETRY_H__
#define __LIBCAMERA_GEOMETRY_H__

#include <string>

namespace libcamera {

struct Rectangle {
	int x;
	int y;
	unsigned int w;
	unsigned int h;

	const std::string toString() const;
};

struct Size {
	Size()
		: Size(0, 0)
	{
	}

	Size(unsigned int w, unsigned int h)
		: width(w), height(h)
	{
	}

	unsigned int width;
	unsigned int height;
};

struct SizeRange {
	SizeRange()
	{
	}

	SizeRange(unsigned int minW, unsigned int minH,
		  unsigned int maxW, unsigned int maxH)
		: min(minW, minH), max(maxW, maxH)
	{
	}

	Size min;
	Size max;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_GEOMETRY_H__ */
