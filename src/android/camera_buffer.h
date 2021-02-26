/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_buffer.h - Frame buffer handling interface definition
 */
#ifndef __ANDROID_CAMERA_BUFFER_H__
#define __ANDROID_CAMERA_BUFFER_H__

#include <hardware/camera3.h>

#include <libcamera/internal/buffer.h>

class CameraBuffer : public libcamera::MappedBuffer
{
public:
	CameraBuffer(buffer_handle_t camera3Buffer, int flags);
	~CameraBuffer();
};

#endif /* __ANDROID_CAMERA_BUFFER_H__ */
