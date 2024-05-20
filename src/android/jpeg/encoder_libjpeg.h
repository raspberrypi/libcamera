/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * JPEG encoding using libjpeg
 */

#pragma once

#include "encoder.h"

#include <vector>

#include "libcamera/internal/formats.h"

#include <jpeglib.h>

class EncoderLibJpeg : public Encoder
{
public:
	EncoderLibJpeg();
	~EncoderLibJpeg();

	int configure(const libcamera::StreamConfiguration &cfg) override;
	int encode(Camera3RequestDescriptor::StreamBuffer *buffer,
		   libcamera::Span<const uint8_t> exifData,
		   unsigned int quality) override;
	int encode(const std::vector<libcamera::Span<uint8_t>> &planes,
		   libcamera::Span<uint8_t> destination,
		   libcamera::Span<const uint8_t> exifData,
		   unsigned int quality);

private:
	void compressRGB(const std::vector<libcamera::Span<uint8_t>> &planes);
	void compressNV(const std::vector<libcamera::Span<uint8_t>> &planes);

	struct jpeg_compress_struct compress_;
	struct jpeg_error_mgr jerr_;

	const libcamera::PixelFormatInfo *pixelFormatInfo_;

	bool nv_;
	bool nvSwap_;
};
