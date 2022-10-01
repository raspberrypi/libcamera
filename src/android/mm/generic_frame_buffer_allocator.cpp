/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * generic_camera_buffer.cpp - Allocate FrameBuffer using gralloc API
 */

#include <memory>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"

#include <hardware/camera3.h>
#include <hardware/gralloc.h>
#include <hardware/hardware.h>

#include "../camera_device.h"
#include "../frame_buffer_allocator.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

namespace {
class GenericFrameBufferData : public FrameBuffer::Private
{
	LIBCAMERA_DECLARE_PUBLIC(FrameBuffer)

public:
	GenericFrameBufferData(struct alloc_device_t *allocDevice,
			       buffer_handle_t handle,
			       const std::vector<FrameBuffer::Plane> &planes)
		: FrameBuffer::Private(planes), allocDevice_(allocDevice),
		  handle_(handle)
	{
		ASSERT(allocDevice_);
		ASSERT(handle_);
	}

	~GenericFrameBufferData() override
	{
		/*
		 * allocDevice_ is used to destroy handle_. allocDevice_ is
		 * owned by PlatformFrameBufferAllocator::Private.
		 * GenericFrameBufferData must be destroyed before it is
		 * destroyed.
		 *
		 * \todo Consider managing alloc_device_t with std::shared_ptr
		 * if this is difficult to maintain.
		 *
		 * \todo Thread safety against alloc_device_t is not documented.
		 * Is it no problem to call alloc/free in parallel?
		 */
		allocDevice_->free(allocDevice_, handle_);
	}

private:
	struct alloc_device_t *allocDevice_;
	const buffer_handle_t handle_;
};
} /* namespace */

class PlatformFrameBufferAllocator::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(PlatformFrameBufferAllocator)

public:
	Private(CameraDevice *const cameraDevice)
		: cameraDevice_(cameraDevice),
		  hardwareModule_(cameraDevice->camera3Device()->common.module),
		  allocDevice_(nullptr)
	{
		ASSERT(hardwareModule_);
	}

	~Private() override;

	std::unique_ptr<libcamera::FrameBuffer>
	allocate(int halPixelFormat, const libcamera::Size &size, uint32_t usage);

private:
	const CameraDevice *const cameraDevice_;
	struct hw_module_t *const hardwareModule_;
	struct alloc_device_t *allocDevice_;
};

PlatformFrameBufferAllocator::Private::~Private()
{
	if (allocDevice_)
		gralloc_close(allocDevice_);
}

std::unique_ptr<libcamera::FrameBuffer>
PlatformFrameBufferAllocator::Private::allocate(int halPixelFormat,
						const libcamera::Size &size,
						uint32_t usage)
{
	if (!allocDevice_) {
		int ret = gralloc_open(hardwareModule_, &allocDevice_);
		if (ret) {
			LOG(HAL, Fatal) << "gralloc_open() failed: " << ret;
			return nullptr;
		}
	}

	int stride = 0;
	buffer_handle_t handle = nullptr;
	int ret = allocDevice_->alloc(allocDevice_, size.width, size.height,
				      halPixelFormat, usage, &handle, &stride);
	if (ret) {
		LOG(HAL, Error) << "failed buffer allocation: " << ret;
		return nullptr;
	}
	if (!handle) {
		LOG(HAL, Fatal) << "invalid buffer_handle_t";
		return nullptr;
	}

	/* This code assumes the planes are mapped consecutively. */
	const libcamera::PixelFormat pixelFormat =
		cameraDevice_->capabilities()->toPixelFormat(halPixelFormat);
	const auto &info = PixelFormatInfo::info(pixelFormat);
	std::vector<FrameBuffer::Plane> planes(info.numPlanes());

	SharedFD fd{ handle->data[0] };
	size_t offset = 0;
	for (auto [i, plane] : utils::enumerate(planes)) {
		const size_t planeSize = info.planeSize(size.height, i, stride);

		plane.fd = fd;
		plane.offset = offset;
		plane.length = planeSize;
		offset += planeSize;
	}

	return std::make_unique<FrameBuffer>(
		std::make_unique<GenericFrameBufferData>(allocDevice_, handle, planes));
}

PUBLIC_FRAME_BUFFER_ALLOCATOR_IMPLEMENTATION
