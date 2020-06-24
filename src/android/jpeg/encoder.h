/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * encoder.h - Image encoding interface
 */
#ifndef __ANDROID_JPEG_ENCODER_H__
#define __ANDROID_JPEG_ENCODER_H__

#include <libcamera/buffer.h>
#include <libcamera/span.h>
#include <libcamera/stream.h>

class Encoder
{
public:
	virtual ~Encoder() {};

	virtual int configure(const libcamera::StreamConfiguration &cfg) = 0;
	virtual int encode(const libcamera::FrameBuffer *source,
			   const libcamera::Span<uint8_t> &destination) = 0;
};

#endif /* __ANDROID_JPEG_ENCODER_H__ */
