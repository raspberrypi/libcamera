/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * generic_camera_buffer.cpp - Generic Android frame buffer backend
 */

#include "../camera_buffer.h"

#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

class CameraBuffer::Private : public Extensible::Private,
			      public libcamera::MappedBuffer
{
	LIBCAMERA_DECLARE_PUBLIC(CameraBuffer)

public:
	Private(CameraBuffer *cameraBuffer, buffer_handle_t camera3Buffer,
		libcamera::PixelFormat pixelFormat, const libcamera::Size &size,
		int flags);
	~Private();

	unsigned int numPlanes() const;

	Span<uint8_t> plane(unsigned int plane);

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;

private:
	/* \todo Remove planes_ when it will be added to MappedBuffer */
	std::vector<Span<uint8_t>> planes_;
};

CameraBuffer::Private::Private([[maybe_unused]] CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer,
			       libcamera::PixelFormat pixelFormat,
			       const libcamera::Size &size, int flags)
{
	error_ = 0;

	const auto &info = libcamera::PixelFormatInfo::info(pixelFormat);
	if (!info.isValid()) {
		error_ = -EINVAL;
		LOG(HAL, Error) << "Invalid pixel format: "
				<< pixelFormat.toString();
		return;
	}

	/*
	 * As Android doesn't offer an API to query buffer layouts, assume for
	 * now that the buffer is backed by a single dmabuf, with planes being
	 * stored contiguously.
	 */
	int fd = -1;
	for (int i = 0; i < camera3Buffer->numFds; i++) {
		if (camera3Buffer->data[i] == -1 || camera3Buffer->data[i] == fd)
			continue;

		if (fd != -1) {
			error_ = -EINVAL;
			LOG(HAL, Error) << "Discontiguous planes are not supported";
			return;
		}

		fd = camera3Buffer->data[i];
	}

	if (fd == -1) {
		error_ = -EINVAL;
		LOG(HAL, Error) << "No valid file descriptor";
		return;
	}

	off_t bufferLength = lseek(fd, 0, SEEK_END);
	if (bufferLength < 0) {
		error_ = -errno;
		LOG(HAL, Error) << "Failed to get buffer length";
		return;
	}

	void *address = mmap(nullptr, bufferLength, flags, MAP_SHARED, fd, 0);
	if (address == MAP_FAILED) {
		error_ = -errno;
		LOG(HAL, Error) << "Failed to mmap plane";
		return;
	}
	maps_.emplace_back(static_cast<uint8_t *>(address), bufferLength);

	const unsigned int numPlanes = info.numPlanes();
	planes_.resize(numPlanes);
	unsigned int offset = 0;
	for (unsigned int i = 0; i < numPlanes; ++i) {
		/*
		 * \todo Remove if this plane size computation function is
		 * added to PixelFormatInfo.
		 */
		const unsigned int vertSubSample = info.planes[i].verticalSubSampling;
		const unsigned int stride = info.stride(size.width, i, 1u);
		const unsigned int planeSize =
			stride * ((size.height + vertSubSample - 1) / vertSubSample);

		planes_[i] = libcamera::Span<uint8_t>(
			static_cast<uint8_t *>(address) + offset, planeSize);

		if (bufferLength < offset + planeSize) {
			error_ = -EINVAL;
			LOG(HAL, Error) << "Plane " << i << " is out of buffer"
					<< ", buffer length=" << bufferLength
					<< ", offset=" << offset
					<< ", size=" << planeSize;
			return;
		}
		offset += planeSize;
	}
}

CameraBuffer::Private::~Private()
{
}

unsigned int CameraBuffer::Private::numPlanes() const
{
	return planes_.size();
}

Span<uint8_t> CameraBuffer::Private::plane(unsigned int plane)
{
	if (plane >= planes_.size())
		return {};

	return planes_[plane];
}

size_t CameraBuffer::Private::jpegBufferSize(size_t maxJpegBufferSize) const
{
	return std::min<unsigned int>(maps_[0].size(),
				      maxJpegBufferSize);
}

PUBLIC_CAMERA_BUFFER_IMPLEMENTATION
