/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * JPEG encoding using CrOS JEA
 */

#include "encoder_jea.h"

#include "libcamera/internal/mapped_framebuffer.h"

#include <cros-camera/camera_mojo_channel_manager_token.h>

#include "../cros_mojo_token.h"
#include "../hal_framebuffer.h"

EncoderJea::EncoderJea() = default;

EncoderJea::~EncoderJea() = default;

int EncoderJea::configure(const libcamera::StreamConfiguration &cfg)
{
	size_ = cfg.size;

	if (jpegCompressor_)
		return 0;

	if (gCrosMojoToken == nullptr)
		return -ENOTSUP;

	jpegCompressor_ = cros::JpegCompressor::GetInstance(gCrosMojoToken);

	return 0;
}

int EncoderJea::encode(Camera3RequestDescriptor::StreamBuffer *buffer,
		       libcamera::Span<const uint8_t> exifData,
		       unsigned int quality)
{
	if (!jpegCompressor_)
		return -ENOTSUP;

	uint32_t outDataSize = 0;
	const HALFrameBuffer *fb =
		dynamic_cast<const HALFrameBuffer *>(buffer->srcBuffer);

	if (!jpegCompressor_->CompressImageFromHandle(fb->handle(),
						      *buffer->camera3Buffer,
						      size_.width, size_.height,
						      quality, exifData.data(),
						      exifData.size(),
						      &outDataSize))
		return -EBUSY;

	return outDataSize;
}
