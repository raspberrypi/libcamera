/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * format_convert.cpp - qcam - Convert buffer to RGB
 */

#include "format_converter.h"

#include <errno.h>
#include <utility>

#include <QImage>

#include <libcamera/formats.h>

#include "../common/image.h"

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

int FormatConverter::configure(const libcamera::PixelFormat &format,
			       const QSize &size, unsigned int stride)
{
	switch (format) {
	case libcamera::formats::NV12:
		formatFamily_ = YUVSemiPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		nvSwap_ = false;
		break;
	case libcamera::formats::NV21:
		formatFamily_ = YUVSemiPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		nvSwap_ = true;
		break;
	case libcamera::formats::NV16:
		formatFamily_ = YUVSemiPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		nvSwap_ = false;
		break;
	case libcamera::formats::NV61:
		formatFamily_ = YUVSemiPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		nvSwap_ = true;
		break;
	case libcamera::formats::NV24:
		formatFamily_ = YUVSemiPlanar;
		horzSubSample_ = 1;
		vertSubSample_ = 1;
		nvSwap_ = false;
		break;
	case libcamera::formats::NV42:
		formatFamily_ = YUVSemiPlanar;
		horzSubSample_ = 1;
		vertSubSample_ = 1;
		nvSwap_ = true;
		break;

	case libcamera::formats::R8:
		formatFamily_ = RGB;
		r_pos_ = 0;
		g_pos_ = 0;
		b_pos_ = 0;
		bpp_ = 1;
		break;
	case libcamera::formats::RGB888:
		formatFamily_ = RGB;
		r_pos_ = 2;
		g_pos_ = 1;
		b_pos_ = 0;
		bpp_ = 3;
		break;
	case libcamera::formats::BGR888:
		formatFamily_ = RGB;
		r_pos_ = 0;
		g_pos_ = 1;
		b_pos_ = 2;
		bpp_ = 3;
		break;
	case libcamera::formats::ARGB8888:
	case libcamera::formats::XRGB8888:
		formatFamily_ = RGB;
		r_pos_ = 2;
		g_pos_ = 1;
		b_pos_ = 0;
		bpp_ = 4;
		break;
	case libcamera::formats::RGBA8888:
	case libcamera::formats::RGBX8888:
		formatFamily_ = RGB;
		r_pos_ = 3;
		g_pos_ = 2;
		b_pos_ = 1;
		bpp_ = 4;
		break;
	case libcamera::formats::ABGR8888:
	case libcamera::formats::XBGR8888:
		formatFamily_ = RGB;
		r_pos_ = 0;
		g_pos_ = 1;
		b_pos_ = 2;
		bpp_ = 4;
		break;
	case libcamera::formats::BGRA8888:
	case libcamera::formats::BGRX8888:
		formatFamily_ = RGB;
		r_pos_ = 1;
		g_pos_ = 2;
		b_pos_ = 3;
		bpp_ = 4;
		break;

	case libcamera::formats::VYUY:
		formatFamily_ = YUVPacked;
		y_pos_ = 1;
		cb_pos_ = 2;
		break;
	case libcamera::formats::YVYU:
		formatFamily_ = YUVPacked;
		y_pos_ = 0;
		cb_pos_ = 3;
		break;
	case libcamera::formats::UYVY:
		formatFamily_ = YUVPacked;
		y_pos_ = 1;
		cb_pos_ = 0;
		break;
	case libcamera::formats::YUYV:
		formatFamily_ = YUVPacked;
		y_pos_ = 0;
		cb_pos_ = 1;
		break;

	case libcamera::formats::YUV420:
		formatFamily_ = YUVPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		nvSwap_ = false;
		break;
	case libcamera::formats::YVU420:
		formatFamily_ = YUVPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		nvSwap_ = true;
		break;
	case libcamera::formats::YUV422:
		formatFamily_ = YUVPlanar;
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		nvSwap_ = false;
		break;

	case libcamera::formats::MJPEG:
		formatFamily_ = MJPEG;
		break;

	default:
		return -EINVAL;
	};

	format_ = format;
	width_ = size.width();
	height_ = size.height();
	stride_ = stride;

	return 0;
}

void FormatConverter::convert(const Image *src, size_t size, QImage *dst)
{
	switch (formatFamily_) {
	case MJPEG:
		dst->loadFromData(src->data(0).data(), size, "JPEG");
		break;
	case RGB:
		convertRGB(src, dst->bits());
		break;
	case YUVPacked:
		convertYUVPacked(src, dst->bits());
		break;
	case YUVSemiPlanar:
		convertYUVSemiPlanar(src, dst->bits());
		break;
	case YUVPlanar:
		convertYUVPlanar(src, dst->bits());
		break;
	};
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

void FormatConverter::convertRGB(const Image *srcImage, unsigned char *dst)
{
	const unsigned char *src = srcImage->data(0).data();
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

		src += stride_;
		dst += width_ * 4;
	}
}

void FormatConverter::convertYUVPacked(const Image *srcImage, unsigned char *dst)
{
	const unsigned char *src = srcImage->data(0).data();
	unsigned int src_x, src_y, dst_x, dst_y;
	unsigned int src_stride;
	unsigned int dst_stride;
	unsigned int cr_pos;
	int r, g, b, y, cr, cb;

	cr_pos = (cb_pos_ + 2) % 4;
	src_stride = stride_;
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

void FormatConverter::convertYUVPlanar(const Image *srcImage, unsigned char *dst)
{
	unsigned int c_stride = stride_ / horzSubSample_;
	unsigned int c_inc = horzSubSample_ == 1 ? 1 : 0;
	const unsigned char *src_y = srcImage->data(0).data();
	const unsigned char *src_cb = srcImage->data(1).data();
	const unsigned char *src_cr = srcImage->data(2).data();
	int r, g, b;

	if (nvSwap_)
		std::swap(src_cb, src_cr);

	for (unsigned int y = 0; y < height_; y++) {
		const unsigned char *line_y = src_y + y * stride_;
		const unsigned char *line_cb = src_cb + (y / vertSubSample_) *
					       c_stride;
		const unsigned char *line_cr = src_cr + (y / vertSubSample_) *
					       c_stride;

		for (unsigned int x = 0; x < width_; x += 2) {
			yuv_to_rgb(*line_y, *line_cb, *line_cr, &r, &g, &b);
			dst[0] = b;
			dst[1] = g;
			dst[2] = r;
			dst[3] = 0xff;
			line_y++;
			line_cb += c_inc;
			line_cr += c_inc;
			dst += 4;

			yuv_to_rgb(*line_y, *line_cb, *line_cr, &r, &g, &b);
			dst[0] = b;
			dst[1] = g;
			dst[2] = r;
			dst[3] = 0xff;
			line_y++;
			line_cb += 1;
			line_cr += 1;
			dst += 4;
		}
	}
}

void FormatConverter::convertYUVSemiPlanar(const Image *srcImage, unsigned char *dst)
{
	unsigned int c_stride = stride_ * (2 / horzSubSample_);
	unsigned int c_inc = horzSubSample_ == 1 ? 2 : 0;
	unsigned int cb_pos = nvSwap_ ? 1 : 0;
	unsigned int cr_pos = nvSwap_ ? 0 : 1;
	const unsigned char *src = srcImage->data(0).data();
	const unsigned char *src_c = srcImage->data(1).data();
	int r, g, b;

	for (unsigned int y = 0; y < height_; y++) {
		const unsigned char *src_y = src + y * stride_;
		const unsigned char *src_cb = src_c + (y / vertSubSample_) *
					      c_stride + cb_pos;
		const unsigned char *src_cr = src_c + (y / vertSubSample_) *
					      c_stride + cr_pos;

		for (unsigned int x = 0; x < width_; x += 2) {
			yuv_to_rgb(*src_y, *src_cb, *src_cr, &r, &g, &b);
			dst[0] = b;
			dst[1] = g;
			dst[2] = r;
			dst[3] = 0xff;
			src_y++;
			src_cb += c_inc;
			src_cr += c_inc;
			dst += 4;

			yuv_to_rgb(*src_y, *src_cb, *src_cr, &r, &g, &b);
			dst[0] = b;
			dst[1] = g;
			dst[2] = r;
			dst[3] = 0xff;
			src_y++;
			src_cb += 2;
			src_cr += 2;
			dst += 4;
		}
	}
}
