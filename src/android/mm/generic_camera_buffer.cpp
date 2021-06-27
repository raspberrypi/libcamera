/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * generic_camera_buffer.cpp - Generic Android frame buffer backend
 */

#include "../camera_buffer.h"

#include <unistd.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/framebuffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

class CameraBuffer::Private : public Extensible::Private,
			      public libcamera::MappedBuffer
{
	LIBCAMERA_DECLARE_PUBLIC(CameraBuffer)

public:
	Private(CameraBuffer *cameraBuffer,
		buffer_handle_t camera3Buffer, int flags);
	~Private();

	unsigned int numPlanes() const;

	Span<uint8_t> plane(unsigned int plane);

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;
};

CameraBuffer::Private::Private(CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer, int flags)
	: Extensible::Private(cameraBuffer)
{
	maps_.reserve(camera3Buffer->numFds);
	error_ = 0;

	for (int i = 0; i < camera3Buffer->numFds; i++) {
		if (camera3Buffer->data[i] == -1)
			continue;

		off_t length = lseek(camera3Buffer->data[i], 0, SEEK_END);
		if (length < 0) {
			error_ = -errno;
			LOG(HAL, Error) << "Failed to query plane length";
			break;
		}

		void *address = mmap(nullptr, length, flags, MAP_SHARED,
				     camera3Buffer->data[i], 0);
		if (address == MAP_FAILED) {
			error_ = -errno;
			LOG(HAL, Error) << "Failed to mmap plane";
			break;
		}

		maps_.emplace_back(static_cast<uint8_t *>(address),
				   static_cast<size_t>(length));
	}
}

CameraBuffer::Private::~Private()
{
}

unsigned int CameraBuffer::Private::numPlanes() const
{
	return maps_.size();
}

Span<uint8_t> CameraBuffer::Private::plane(unsigned int plane)
{
	if (plane >= maps_.size())
		return {};

	return maps_[plane];
}

size_t CameraBuffer::Private::jpegBufferSize(size_t maxJpegBufferSize) const
{
	return std::min<unsigned int>(maps_[0].size(),
				      maxJpegBufferSize);
}

PUBLIC_CAMERA_BUFFER_IMPLEMENTATION
