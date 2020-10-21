/* SPDX-License-Identifier: LGPL-2.1-or-later */
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
	int encode(const libcamera::FrameBuffer &source,
		   libcamera::Span<uint8_t> destination,
		   const libcamera::Span<const uint8_t> &exifData) override;

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
