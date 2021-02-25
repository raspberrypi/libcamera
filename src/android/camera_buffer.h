/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_buffer.h - Frame buffer handling interface definition
 */
#ifndef __ANDROID_CAMERA_BUFFER_H__
#define __ANDROID_CAMERA_BUFFER_H__

#include <hardware/camera3.h>

#include <libcamera/class.h>
#include <libcamera/span.h>

class CameraBuffer final : public libcamera::Extensible
{
	LIBCAMERA_DECLARE_PRIVATE(CameraBuffer)

public:
	CameraBuffer(buffer_handle_t camera3Buffer, int flags);
	~CameraBuffer();

	bool isValid() const;

	unsigned int numPlanes() const;

	libcamera::Span<const uint8_t> plane(unsigned int plane) const;
	libcamera::Span<uint8_t> plane(unsigned int plane);

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;
};

#endif /* __ANDROID_CAMERA_BUFFER_H__ */
