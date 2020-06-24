/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * encoder_libjpeg.h - JPEG encoding using libjpeg
 */
#ifndef __ANDROID_JPEG_ENCODER_LIBJPEG_H__
#define __ANDROID_JPEG_ENCODER_LIBJPEG_H__

#include "encoder.h"

#include "libcamera/internal/buffer.h"
#include "libcamera/internal/formats.h"

#include <jpeglib.h>

class EncoderLibJpeg : public Encoder
{
public:
	EncoderLibJpeg();
	~EncoderLibJpeg();

	int configure(const libcamera::StreamConfiguration &cfg) override;
	int encode(const libcamera::FrameBuffer *source,
		   const libcamera::Span<uint8_t> &destination) override;

private:
	void compressRGB(const libcamera::MappedBuffer *frame);
	void compressNV(const libcamera::MappedBuffer *frame);

	struct jpeg_compress_struct compress_;
	struct jpeg_error_mgr jerr_;

	unsigned int quality_;

	const libcamera::PixelFormatInfo *pixelFormatInfo_;

	bool nv_;
	bool nvSwap_;
};

#endif /* __ANDROID_JPEG_ENCODER_LIBJPEG_H__ */
