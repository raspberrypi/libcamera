/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Frame buffer handling interface definition
 */

#pragma once

#include <hardware/camera3.h>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

class CameraBuffer final : public libcamera::Extensible
{
	LIBCAMERA_DECLARE_PRIVATE()

public:
	CameraBuffer(buffer_handle_t camera3Buffer,
		     libcamera::PixelFormat pixelFormat,
		     const libcamera::Size &size, int flags);
	~CameraBuffer();

	bool isValid() const;

	unsigned int numPlanes() const;

	libcamera::Span<const uint8_t> plane(unsigned int plane) const;
	libcamera::Span<uint8_t> plane(unsigned int plane);

	unsigned int stride(unsigned int plane) const;
	unsigned int offset(unsigned int plane) const;
	unsigned int size(unsigned int plane) const;

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;
};

#define PUBLIC_CAMERA_BUFFER_IMPLEMENTATION				\
CameraBuffer::CameraBuffer(buffer_handle_t camera3Buffer,		\
			   libcamera::PixelFormat pixelFormat,		\
			   const libcamera::Size &size, int flags)	\
	: Extensible(std::make_unique<Private>(this, camera3Buffer,	\
					       pixelFormat, size,	\
					       flags))			\
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
unsigned int CameraBuffer::stride(unsigned int plane) const		\
{									\
	return _d()->stride(plane);					\
}									\
unsigned int CameraBuffer::offset(unsigned int plane) const		\
{									\
	return _d()->offset(plane);					\
}									\
unsigned int CameraBuffer::size(unsigned int plane) const		\
{									\
	return _d()->size(plane);					\
}									\
size_t CameraBuffer::jpegBufferSize(size_t maxJpegBufferSize) const	\
{									\
	return _d()->jpegBufferSize(maxJpegBufferSize);			\
}
