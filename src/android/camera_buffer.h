/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_buffer.h - Frame buffer handling interface definition
 */
#ifndef __ANDROID_CAMERA_BUFFER_H__
#define __ANDROID_CAMERA_BUFFER_H__

#include <hardware/camera3.h>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

class CameraBuffer final : public libcamera::Extensible
{
	LIBCAMERA_DECLARE_PRIVATE()

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
	: Extensible(std::make_unique<Private>(this, camera3Buffer, flags)) \
{									\
}									\
CameraBuffer::~CameraBuffer()						\
{									\
}									\
bool CameraBuffer::isValid() const					\
{									\
	return _d()->isValid();						\
}									\
unsigned int CameraBuffer::numPlanes() const				\
{									\
	return _d()->numPlanes();					\
}									\
Span<const uint8_t> CameraBuffer::plane(unsigned int plane) const	\
{									\
	return const_cast<Private *>(_d())->plane(plane);		\
}									\
Span<uint8_t> CameraBuffer::plane(unsigned int plane)			\
{									\
	return _d()->plane(plane);					\
}									\
size_t CameraBuffer::jpegBufferSize(size_t maxJpegBufferSize) const	\
{									\
	return _d()->jpegBufferSize(maxJpegBufferSize);			\
}
#endif /* __ANDROID_CAMERA_BUFFER_H__ */
