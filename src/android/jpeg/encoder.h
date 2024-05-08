/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image encoding interface
 */

#pragma once

#include <libcamera/base/span.h>

#include <libcamera/framebuffer.h>
#include <libcamera/stream.h>

#include "../camera_request.h"

class Encoder
{
public:
	virtual ~Encoder() = default;

	virtual int configure(const libcamera::StreamConfiguration &cfg) = 0;
	virtual int encode(Camera3RequestDescriptor::StreamBuffer *buffer,
			   libcamera::Span<const uint8_t> exifData,
			   unsigned int quality) = 0;
};
