/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * JPEG encoding using CrOS JEA
 */

#pragma once

#include <libcamera/geometry.h>

#include <cros-camera/jpeg_compressor.h>

#include "encoder.h"

class EncoderJea : public Encoder
{
public:
	EncoderJea();
	~EncoderJea();

	int configure(const libcamera::StreamConfiguration &cfg) override;
	int encode(Camera3RequestDescriptor::StreamBuffer *buffer,
		   libcamera::Span<const uint8_t> exifData,
		   unsigned int quality) override;

private:
	libcamera::Size size_;

	std::unique_ptr<cros::JpegCompressor> jpegCompressor_;
};
