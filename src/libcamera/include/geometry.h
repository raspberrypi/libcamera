/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.h - Geometry-related structure
 */

#ifndef __LIBCAMERA_GEOMETRY_H__
#define __LIBCAMERA_GEOMETRY_H__

namespace libcamera {

struct Rectangle {
	int x;
	int y;
	unsigned int w;
	unsigned int h;
};

struct SizeRange {
	SizeRange(unsigned int minW, unsigned int minH,
		  unsigned int maxW, unsigned int maxH)
		: minWidth(minW), minHeight(minH), maxWidth(maxW),
		  maxHeight(maxH) {}

	unsigned int minWidth;
	unsigned int minHeight;
	unsigned int maxWidth;
	unsigned int maxHeight;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_GEOMETRY_H__ */
