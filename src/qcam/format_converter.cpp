/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * format_convert.cpp - qcam - Convert buffer to RGB
 */

#include <errno.h>

#include <linux/videodev2.h>

#include <QImage>

#include "format_converter.h"

#define RGBSHIFT		8
#ifndef MAX
#define MAX(a,b)		((a)>(b)?(a):(b))
#endif
#ifndef MIN
#define MIN(a,b)		((a)<(b)?(a):(b))
#endif
#ifndef CLAMP
#define CLAMP(a,low,high)	MAX((low),MIN((high),(a)))
#endif
#ifndef CLIP
#define CLIP(x)			CLAMP(x,0,255)
#endif

int FormatConverter::configure(unsigned int format, unsigned int width,
			       unsigned int height)
{
	switch (format) {
	case V4L2_PIX_FMT_BGR24:
		formatFamily_ = RGB;
		r_pos_ = 2;
		g_pos_ = 1;
		b_pos_ = 0;
		bpp_ = 3;
		break;
	case V4L2_PIX_FMT_RGB24:
		formatFamily_ = RGB;
		r_pos_ = 0;
		g_pos_ = 1;
		b_pos_ = 2;
		bpp_ = 3;
		break;
	case V4L2_PIX_FMT_ARGB32:
		formatFamily_ = RGB;
		r_pos_ = 1;
		g_pos_ = 2;
		b_pos_ = 3;
		bpp_ = 4;
		break;
	case V4L2_PIX_FMT_VYUY:
		formatFamily_ = YUV;
		y_pos_ = 1;
		cb_pos_ = 2;
		break;
	case V4L2_PIX_FMT_YVYU:
		formatFamily_ = YUV;
		y_pos_ = 0;
		cb_pos_ = 3;
		break;
	case V4L2_PIX_FMT_UYVY:
		formatFamily_ = YUV;
		y_pos_ = 1;
		cb_pos_ = 0;
		break;
	case V4L2_PIX_FMT_YUYV:
		formatFamily_ = YUV;
		y_pos_ = 0;
		cb_pos_ = 1;
		break;
	case V4L2_PIX_FMT_MJPEG:
		formatFamily_ = MJPEG;
		break;
	default:
		return -EINVAL;
	};

	format_ = format;
	width_ = width;
	height_ = height;

	return 0;
}

void FormatConverter::convert(const unsigned char *src, size_t size,
			      QImage *dst)
{
	switch (formatFamily_) {
	case MJPEG:
		dst->loadFromData(src, size, "JPEG");
		break;
	case YUV:
		convertYUV(src, dst->bits());
		break;
	case RGB:
		convertRGB(src, dst->bits());
		break;
	};
}

void FormatConverter::convertRGB(const unsigned char *src, unsigned char *dst)
{
	unsigned int x, y;
	int r, g, b;

	for (y = 0; y < height_; y++) {
		for (x = 0; x < width_; x++) {
			r = src[bpp_ * x + r_pos_];
			g = src[bpp_ * x + g_pos_];
			b = src[bpp_ * x + b_pos_];

			dst[4 * x + 0] = b;
			dst[4 * x + 1] = g;
			dst[4 * x + 2] = r;
			dst[4 * x + 3] = 0xff;
		}

		src += width_ * bpp_;
		dst += width_ * 4;
	}
}

static void yuv_to_rgb(int y, int u, int v, int *r, int *g, int *b)
{
	int c = y - 16;
	int d = u - 128;
	int e = v - 128;
	*r = CLIP(( 298 * c           + 409 * e + 128) >> RGBSHIFT);
	*g = CLIP(( 298 * c - 100 * d - 208 * e + 128) >> RGBSHIFT);
	*b = CLIP(( 298 * c + 516 * d           + 128) >> RGBSHIFT);
}

void FormatConverter::convertYUV(const unsigned char *src, unsigned char *dst)
{
	unsigned int src_x, src_y, dst_x, dst_y;
	unsigned int src_stride;
	unsigned int dst_stride;
	unsigned int cr_pos;
	int r, g, b, y, cr, cb;

	cr_pos = (cb_pos_ + 2) % 4;
	src_stride = width_ * 2;
	dst_stride = width_ * 4;

	for (src_y = 0, dst_y = 0; dst_y < height_; src_y++, dst_y++) {
		for (src_x = 0, dst_x = 0; dst_x < width_; ) {
			cb = src[src_y * src_stride + src_x * 4 + cb_pos_];
			cr = src[src_y * src_stride + src_x * 4 + cr_pos];

			y = src[src_y * src_stride + src_x * 4 + y_pos_];
			yuv_to_rgb(y, cb, cr, &r, &g, &b);
			dst[dst_y * dst_stride + 4 * dst_x + 0] = b;
			dst[dst_y * dst_stride + 4 * dst_x + 1] = g;
			dst[dst_y * dst_stride + 4 * dst_x + 2] = r;
			dst[dst_y * dst_stride + 4 * dst_x + 3] = 0xff;
			dst_x++;

			y = src[src_y * src_stride + src_x * 4 + y_pos_ + 2];
			yuv_to_rgb(y, cb, cr, &r, &g, &b);
			dst[dst_y * dst_stride + 4 * dst_x + 0] = b;
			dst[dst_y * dst_stride + 4 * dst_x + 1] = g;
			dst[dst_y * dst_stride + 4 * dst_x + 2] = r;
			dst[dst_y * dst_stride + 4 * dst_x + 3] = 0xff;
			dst_x++;

			src_x++;
		}
	}
}
