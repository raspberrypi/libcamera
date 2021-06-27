/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * framebuffer.h - Internal frame buffer handling
 */
#ifndef __LIBCAMERA_INTERNAL_FRAMEBUFFER_H__
#define __LIBCAMERA_INTERNAL_FRAMEBUFFER_H__

#include <sys/mman.h>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

#include <libcamera/framebuffer.h>

namespace libcamera {

class MappedBuffer
{
public:
	using Plane = Span<uint8_t>;

	~MappedBuffer();

	MappedBuffer(MappedBuffer &&other);
	MappedBuffer &operator=(MappedBuffer &&other);

	bool isValid() const { return error_ == 0; }
	int error() const { return error_; }
	const std::vector<Plane> &maps() const { return maps_; }

protected:
	MappedBuffer();

	int error_;
	std::vector<Plane> maps_;

private:
	LIBCAMERA_DISABLE_COPY(MappedBuffer)
};

class MappedFrameBuffer : public MappedBuffer
{
public:
	MappedFrameBuffer(const FrameBuffer *buffer, int flags);
};

class FrameBuffer::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(FrameBuffer)

public:
	Private(FrameBuffer *buffer);

	void setRequest(Request *request) { request_ = request; }

private:
	Request *request_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_FRAMEBUFFER_H__ */
