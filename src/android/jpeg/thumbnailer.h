/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * thumbnailer.h - Simple image thumbnailer
 */
#ifndef __ANDROID_JPEG_THUMBNAILER_H__
#define __ANDROID_JPEG_THUMBNAILER_H__

#include <libcamera/geometry.h>

#include "libcamera/internal/buffer.h"
#include "libcamera/internal/formats.h"

class Thumbnailer
{
public:
	Thumbnailer();

	void configure(const libcamera::Size &sourceSize,
		       libcamera::PixelFormat pixelFormat);
	void createThumbnail(const libcamera::FrameBuffer &source,
			     std::vector<unsigned char> *dest);
	const libcamera::Size &size() const { return targetSize_; }

private:
	libcamera::Size computeThumbnailSize() const;

	libcamera::PixelFormat pixelFormat_;
	libcamera::Size sourceSize_;
	libcamera::Size targetSize_;

	bool valid_;
};

#endif /* __ANDROID_JPEG_THUMBNAILER_H__ */
