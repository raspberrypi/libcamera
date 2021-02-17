/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * generic_camera_buffer.cpp - Generic Android frame buffer backend
 */

#include "../camera_buffer.h"

#include "libcamera/internal/log.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

CameraBuffer::CameraBuffer(const buffer_handle_t camera3buffer, int flags)
{
	maps_.reserve(camera3buffer->numFds);
	error_ = 0;

	for (int i = 0; i < camera3buffer->numFds; i++) {
		if (camera3buffer->data[i] == -1)
			continue;

		off_t length = lseek(camera3buffer->data[i], 0, SEEK_END);
		if (length < 0) {
			error_ = -errno;
			LOG(HAL, Error) << "Failed to query plane length";
			break;
		}

		void *address = mmap(nullptr, length, flags, MAP_SHARED,
				     camera3buffer->data[i], 0);
		if (address == MAP_FAILED) {
			error_ = -errno;
			LOG(HAL, Error) << "Failed to mmap plane";
			break;
		}

		maps_.emplace_back(static_cast<uint8_t *>(address),
				   static_cast<size_t>(length));
	}
}

CameraBuffer::~CameraBuffer()
{
}
