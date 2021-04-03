/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * cros_camera_buffer.cpp - Chromium OS buffer backend using CameraBufferManager
 */

#include "../camera_buffer.h"

#include "libcamera/internal/log.h"

#include "cros-camera/camera_buffer_manager.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

class CameraBuffer::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(CameraBuffer)

public:
	Private(CameraBuffer *cameraBuffer,
		buffer_handle_t camera3Buffer, int flags);
	~Private();

	bool isValid() const { return valid_; }

	unsigned int numPlanes() const;

	Span<uint8_t> plane(unsigned int plane);

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;

private:
	cros::CameraBufferManager *bufferManager_;
	buffer_handle_t handle_;
	unsigned int numPlanes_;
	bool valid_;
	bool registered_;
	union {
		void *addr;
		android_ycbcr ycbcr;
	} mem;

	const uint8_t *planeAddr(unsigned int plane) const;
	uint8_t *planeAddr(unsigned int plane);
};

CameraBuffer::Private::Private(CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer, int flags)
	: Extensible::Private(cameraBuffer), handle_(camera3Buffer),
	  numPlanes_(0), valid_(false), registered(false)
{
	bufferManager_ = cros::CameraBufferManager::GetInstance();

	int ret = bufferManager_->Register(camera3Buffer);
	if (ret) {
		LOG(HAL, Error) << "Failed registering a buffer: " << ret;
		return;
	}

	registered_ = true;
	numPlanes_ = bufferManager_->GetNumPlanes(camera3Buffer);
	switch (numPlanes_) {
	case 1: {
		ret = bufferManager_->Lock(handle_, 0, 0, 0, 0, 0, &mem.addr);
		if (ret) {
			LOG(HAL, Error) << "Single plane buffer mapping failed";
			return;
		}
		break;
	}
	case 2:
	case 3: {
		ret = bufferManager_->LockYCbCr(handle_, 0, 0, 0, 0, 0,
						&mem.ycbcr);
		if (ret) {
			LOG(HAL, Error) << "YCbCr buffer mapping failed";
			return;
		}
		break;
	}
	default:
		LOG(HAL, Error) << "Invalid number of planes: " << numPlanes_;
		return;
	}

	valid_ = true;
}

CameraBuffer::Private::~Private()
{
	if (valid_)
		bufferManager_->Unlock(handle_);
	if (registered_)
		bufferManager_->Deregister(handle_);
}

unsigned int CameraBuffer::Private::numPlanes() const
{
	return bufferManager_->GetNumPlanes(handle_);
}

Span<uint8_t> CameraBuffer::Private::plane(unsigned int plane)
{
	void *addr;

	switch (numPlanes()) {
	case 1:
		addr = mem.addr;
		break;
	default:
		switch (plane) {
		case 1:
			addr = mem.ycbcr.y;
			break;
		case 2:
			addr = mem.ycbcr.cb;
			break;
		case 3:
			addr = mem.ycbcr.cr;
			break;
		}
	}

	return { static_cast<uint8_t *>(addr),
		 bufferManager_->GetPlaneSize(handle_, plane) };
}

size_t CameraBuffer::Private::jpegBufferSize(size_t maxJpegBufferSize) const
{
	return bufferManager_->GetPlaneSize(handle_, 0);
}

PUBLIC_CAMERA_BUFFER_IMPLEMENTATION
