/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * format_convert.h - qcam - Convert buffer to RGB
 */
#ifndef __QCAM_FORMAT_CONVERTER_H__
#define __QCAM_FORMAT_CONVERTER_H__

#include <stddef.h>

class QImage;

class FormatConverter
{
public:
	int configure(unsigned int format, unsigned int width,
		      unsigned int height);

	void convert(const unsigned char *src, size_t size, QImage *dst);

private:
	enum FormatFamily {
		MJPEG,
		NV,
		RGB,
		YUV,
	};

	void convertNV(const unsigned char *src, unsigned char *dst);
	void convertRGB(const unsigned char *src, unsigned char *dst);
	void convertYUV(const unsigned char *src, unsigned char *dst);

	unsigned int format_;
	unsigned int width_;
	unsigned int height_;

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

#endif /* __QCAM_FORMAT_CONVERTER_H__ */
