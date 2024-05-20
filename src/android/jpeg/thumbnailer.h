/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Simple image thumbnailer
 */

#pragma once

#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/formats.h"

class Thumbnailer
{
public:
	Thumbnailer();

	void configure(const libcamera::Size &sourceSize,
		       libcamera::PixelFormat pixelFormat);
	void createThumbnail(const libcamera::FrameBuffer &source,
			     const libcamera::Size &targetSize,
			     std::vector<unsigned char> *dest);
	const libcamera::PixelFormat &pixelFormat() const { return pixelFormat_; }

private:
	libcamera::PixelFormat pixelFormat_;
	libcamera::Size sourceSize_;

	bool valid_;
};
