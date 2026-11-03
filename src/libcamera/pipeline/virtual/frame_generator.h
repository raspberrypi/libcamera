/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Virtual cameras helper to generate frames
 */

#pragma once

#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

namespace libcamera {

class FrameGenerator
{
public:
	virtual ~FrameGenerator() = default;

	virtual void configure(const Size &size) = 0;

	virtual int generateFrame(const StreamConfiguration &cfg,
				  const FrameBuffer *buffer) = 0;

protected:
	FrameGenerator() {}
};

} /* namespace libcamera */
