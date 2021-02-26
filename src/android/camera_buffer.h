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

#define PUBLIC_CAMERA_BUFFER_IMPLEMENTATION				\
CameraBuffer::CameraBuffer(buffer_handle_t camera3Buffer, int flags)	\
	: Extensible(new Private(this, camera3Buffer, flags))		\
{									\
}									\
CameraBuffer::~CameraBuffer()						\
{									\
}									\
bool CameraBuffer::isValid() const					\
{									\
	const Private *const d = LIBCAMERA_D_PTR();			\
	return d->isValid();						\
}									\
unsigned int CameraBuffer::numPlanes() const				\
{									\
	const Private *const d = LIBCAMERA_D_PTR();			\
	return d->numPlanes();						\
}									\
Span<const uint8_t> CameraBuffer::plane(unsigned int plane) const	\
{									\
	const Private *const d = LIBCAMERA_D_PTR();			\
	return const_cast<Private *>(d)->plane(plane);			\
}									\
Span<uint8_t> CameraBuffer::plane(unsigned int plane)			\
{									\
	Private *const d = LIBCAMERA_D_PTR();				\
	return d->plane(plane);						\
}									\
size_t CameraBuffer::jpegBufferSize(size_t maxJpegBufferSize) const	\
{									\
	const Private *const d = LIBCAMERA_D_PTR();			\
	return d->jpegBufferSize(maxJpegBufferSize);			\
}
#endif /* __ANDROID_CAMERA_BUFFER_H__ */
