/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Interface definition to allocate Frame buffer in
 * platform dependent way.
 */
#ifndef __ANDROID_FRAME_BUFFER_ALLOCATOR_H__
#define __ANDROID_FRAME_BUFFER_ALLOCATOR_H__

#include <memory>

#include <libcamera/base/class.h>

#include <libcamera/camera.h>
#include <libcamera/geometry.h>

#include "hal_framebuffer.h"

class CameraDevice;

class PlatformFrameBufferAllocator : libcamera::Extensible
{
	LIBCAMERA_DECLARE_PRIVATE()

public:
	explicit PlatformFrameBufferAllocator(CameraDevice *const cameraDevice);
	~PlatformFrameBufferAllocator();

	/*
	 * FrameBuffer owns the underlying buffer. Returns nullptr on failure.
	 * Note: The returned FrameBuffer needs to be destroyed before
	 * PlatformFrameBufferAllocator is destroyed.
	 */
	std::unique_ptr<HALFrameBuffer> allocate(
		int halPixelFormat, const libcamera::Size &size, uint32_t usage);
};

#define PUBLIC_FRAME_BUFFER_ALLOCATOR_IMPLEMENTATION			\
PlatformFrameBufferAllocator::PlatformFrameBufferAllocator(		\
	CameraDevice *const cameraDevice)				\
	: Extensible(std::make_unique<Private>(cameraDevice))		\
{									\
}									\
PlatformFrameBufferAllocator::~PlatformFrameBufferAllocator()		\
{									\
}									\
std::unique_ptr<HALFrameBuffer> 					\
PlatformFrameBufferAllocator::allocate(int halPixelFormat,		\
				       const libcamera::Size &size,	\
				       uint32_t usage)			\
{									\
	return _d()->allocate(halPixelFormat, size, usage);		\
}

#endif /* __ANDROID_FRAME_BUFFER_ALLOCATOR_H__ */
