/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * cros_frame_buffer.cpp - Allocate FrameBuffer for Chromium OS using
 * CameraBufferManager
 */

#include <memory>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>

#include "libcamera/internal/framebuffer.h"

#include "../camera_device.h"
#include "../frame_buffer_allocator.h"
#include "cros-camera/camera_buffer_manager.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {
class CrosFrameBufferData : public FrameBuffer::Private
{
	LIBCAMERA_DECLARE_PUBLIC(FrameBuffer)

public:
	CrosFrameBufferData(cros::ScopedBufferHandle scopedHandle,
			    const std::vector<FrameBuffer::Plane> &planes)
		: FrameBuffer::Private(planes), scopedHandle_(std::move(scopedHandle))
	{
	}

private:
	cros::ScopedBufferHandle scopedHandle_;
};
} /* namespace */

class PlatformFrameBufferAllocator::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(PlatformFrameBufferAllocator)

public:
	Private([[maybe_unused]] CameraDevice *const cameraDevice)
	{
	}

	std::unique_ptr<libcamera::FrameBuffer>
	allocate(int halPixelFormat, const libcamera::Size &size, uint32_t usage);
};

std::unique_ptr<libcamera::FrameBuffer>
PlatformFrameBufferAllocator::Private::allocate(int halPixelFormat,
						const libcamera::Size &size,
						uint32_t usage)
{
	cros::ScopedBufferHandle scopedHandle =
		cros::CameraBufferManager::AllocateScopedBuffer(
			size.width, size.height, halPixelFormat, usage);
	if (!scopedHandle) {
		LOG(HAL, Error) << "Failed to allocate buffer handle";
		return nullptr;
	}

	buffer_handle_t handle = *scopedHandle;
	SharedFD fd{ handle->data[0] };
	if (!fd.isValid()) {
		LOG(HAL, Fatal) << "Invalid fd";
		return nullptr;
	}

	/* This code assumes all the planes are located in the same buffer. */
	const size_t numPlanes = cros::CameraBufferManager::GetNumPlanes(handle);
	std::vector<FrameBuffer::Plane> planes(numPlanes);
	for (auto [i, plane] : utils::enumerate(planes)) {
		plane.fd = fd;
		plane.offset = cros::CameraBufferManager::GetPlaneOffset(handle, i);
		plane.length = cros::CameraBufferManager::GetPlaneSize(handle, i);
	}

	return std::make_unique<FrameBuffer>(
		std::make_unique<CrosFrameBufferData>(std::move(scopedHandle), planes));
}

PUBLIC_FRAME_BUFFER_ALLOCATOR_IMPLEMENTATION
