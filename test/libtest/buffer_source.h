/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * buffer_source.h - libcamera camera test helper to create FrameBuffers
 */
#ifndef __LIBCAMERA_BUFFER_SOURCE_TEST_H__
#define __LIBCAMERA_BUFFER_SOURCE_TEST_H__

#include <libcamera/libcamera.h>

#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

using namespace libcamera;

class BufferSource
{
public:
	BufferSource();
	~BufferSource();

	int allocate(const StreamConfiguration &config);
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers();

private:
	std::shared_ptr<MediaDevice> media_;
	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
};

#endif /* __LIBCAMERA_BUFFER_SOURCE_TEST_H__ */
