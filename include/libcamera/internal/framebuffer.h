/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * framebuffer.h - Internal frame buffer handling
 */
#ifndef __LIBCAMERA_INTERNAL_FRAMEBUFFER_H__
#define __LIBCAMERA_INTERNAL_FRAMEBUFFER_H__

#include <libcamera/base/class.h>

#include <libcamera/framebuffer.h>

namespace libcamera {

class FrameBuffer::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(FrameBuffer)

public:
	Private();

	void setRequest(Request *request) { request_ = request; }
	bool isContiguous() const { return isContiguous_; }

private:
	Request *request_;
	bool isContiguous_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_FRAMEBUFFER_H__ */
