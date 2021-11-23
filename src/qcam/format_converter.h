/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * format_convert.h - qcam - Convert buffer to RGB
 */

#pragma once

#include <stddef.h>

#include <QSize>

#include <libcamera/pixel_format.h>

class Image;
class QImage;

class FormatConverter
{
public:
	int configure(const libcamera::PixelFormat &format, const QSize &size,
		      unsigned int stride);

	void convert(const Image *src, size_t size, QImage *dst);

private:
	enum FormatFamily {
		MJPEG,
		RGB,
		YUVPacked,
		YUVPlanar,
		YUVSemiPlanar,
	};

	void convertRGB(const Image *src, unsigned char *dst);
	void convertYUVPacked(const Image *src, unsigned char *dst);
	void convertYUVPlanar(const Image *src, unsigned char *dst);
	void convertYUVSemiPlanar(const Image *src, unsigned char *dst);

	libcamera::PixelFormat format_;
	unsigned int width_;
	unsigned int height_;
	unsigned int stride_;

	enum FormatFamily formatFamily_;

	/* NV parameters */
	unsigned int horzSubSample_;
	unsigned int vertSubSample_;
	bool nvSwap_;

	/* RGB parameters */
	unsigned int bpp_;
	unsigned int r_pos_;
	unsigned int g_pos_;
	unsigned int b_pos_;

	/* YUV parameters */
	unsigned int y_pos_;
	unsigned int cb_pos_;
};
