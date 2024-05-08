/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * HAL Frame Buffer Handling
 */

#pragma once

#include "libcamera/internal/framebuffer.h"

#include <hardware/camera3.h>

class HALFrameBuffer final : public libcamera::FrameBuffer
{
public:
	HALFrameBuffer(std::unique_ptr<Private> d,
		       buffer_handle_t handle);
	HALFrameBuffer(const std::vector<Plane> &planes,
		       buffer_handle_t handle);

	buffer_handle_t handle() const { return handle_; }

private:
	buffer_handle_t handle_;
};
