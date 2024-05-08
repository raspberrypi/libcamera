/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * HAL Frame Buffer Handling
 */

#include "hal_framebuffer.h"

#include <hardware/camera3.h>

HALFrameBuffer::HALFrameBuffer(std::unique_ptr<Private> d,
			       buffer_handle_t handle)
	: FrameBuffer(std::move(d)), handle_(handle)
{
}

HALFrameBuffer::HALFrameBuffer(const std::vector<Plane> &planes,
			       buffer_handle_t handle)
	: FrameBuffer(planes), handle_(handle)
{
}
