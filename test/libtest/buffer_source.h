/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * buffer_source.h - libcamera camera test helper to create FrameBuffers
 */

#pragma once

#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

class BufferSource
{
public:
	BufferSource();
	~BufferSource();

	int allocate(const libcamera::StreamConfiguration &config);
	const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers();

private:
	std::shared_ptr<libcamera::MediaDevice> media_;
	std::vector<std::unique_ptr<libcamera::FrameBuffer>> buffers_;
};
