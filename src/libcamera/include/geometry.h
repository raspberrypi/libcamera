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

} /* namespace libcamera */

#endif /* __LIBCAMERA_GEOMETRY_H__ */
