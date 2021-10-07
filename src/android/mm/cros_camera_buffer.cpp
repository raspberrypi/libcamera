/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * cros_camera_buffer.cpp - Chromium OS buffer backend using CameraBufferManager
 */

#include "../camera_buffer.h"

#include <libcamera/base/log.h>

#include "cros-camera/camera_buffer_manager.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

class CameraBuffer::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(CameraBuffer)

public:
	Private(CameraBuffer *cameraBuffer, buffer_handle_t camera3Buffer,
		PixelFormat pixelFormat, const Size &size,
		int flags);
	~Private();

	bool isValid() const { return registered_; }

	unsigned int numPlanes() const;

	Span<uint8_t> plane(unsigned int plane);

	unsigned int stride(unsigned int plane) const;
	unsigned int offset(unsigned int plane) const;
	unsigned int size(unsigned int plane) const;

	size_t jpegBufferSize(size_t maxJpegBufferSize) const;

private:
	void map();

	cros::CameraBufferManager *bufferManager_;
	buffer_handle_t handle_;
	unsigned int numPlanes_;
	bool mapped_;
	bool registered_;
	union {
		void *addr;
		android_ycbcr ycbcr;
	} mem;
};

CameraBuffer::Private::Private([[maybe_unused]] CameraBuffer *cameraBuffer,
			       buffer_handle_t camera3Buffer,
			       [[maybe_unused]] PixelFormat pixelFormat,
			       [[maybe_unused]] const Size &size,
			       [[maybe_unused]] int flags)
	: handle_(camera3Buffer), numPlanes_(0), mapped_(false),
	  registered_(false)
{
	bufferManager_ = cros::CameraBufferManager::GetInstance();
	if (!bufferManager_) {
		LOG(HAL, Fatal)
			<< "Failed to get cros CameraBufferManager instance";
		return;
	}

	int ret = bufferManager_->Register(camera3Buffer);
	if (ret) {
		LOG(HAL, Error) << "Failed registering a buffer: " << ret;
		return;
	}

	registered_ = true;
	numPlanes_ = bufferManager_->GetNumPlanes(camera3Buffer);
}

CameraBuffer::Private::~Private()
{
	int ret;
	if (mapped_) {
		ret = bufferManager_->Unlock(handle_);
		if (ret != 0)
			LOG(HAL, Error) << "Failed to unlock buffer: "
					<< strerror(-ret);
	}

	if (registered_) {
		ret = bufferManager_->Deregister(handle_);
		if (ret != 0)
			LOG(HAL, Error) << "Failed to deregister buffer: "
					<< strerror(-ret);
	}
}

unsigned int CameraBuffer::Private::numPlanes() const
{
	return bufferManager_->GetNumPlanes(handle_);
}

Span<uint8_t> CameraBuffer::Private::plane(unsigned int plane)
{
	if (!mapped_)
		map();
	if (!mapped_)
		return {};

	void *addr;

	switch (numPlanes()) {
	case 1:
		addr = mem.addr;
		break;
	default:
		switch (plane) {
		case 0:
			addr = mem.ycbcr.y;
			break;
		case 1:
			addr = mem.ycbcr.cb;
			break;
		case 2:
			addr = mem.ycbcr.cr;
			break;
		}
	}

	return { static_cast<uint8_t *>(addr),
		 bufferManager_->GetPlaneSize(handle_, plane) };
}

unsigned int CameraBuffer::Private::stride(unsigned int plane) const
{
	return cros::CameraBufferManager::GetPlaneStride(handle_, plane);
}

unsigned int CameraBuffer::Private::offset(unsigned int plane) const
{
	return cros::CameraBufferManager::GetPlaneOffset(handle_, plane);
}

unsigned int CameraBuffer::Private::size(unsigned int plane) const
{
	return cros::CameraBufferManager::GetPlaneSize(handle_, plane);
}

size_t CameraBuffer::Private::jpegBufferSize([[maybe_unused]] size_t maxJpegBufferSize) const
{
	return bufferManager_->GetPlaneSize(handle_, 0);
}

void CameraBuffer::Private::map()
{
	int ret;
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

	mapped_ = true;
	return;
}

PUBLIC_CAMERA_BUFFER_IMPLEMENTATION
