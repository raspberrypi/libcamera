/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * buffer.h - Internal buffer handling
 */
#ifndef __LIBCAMERA_INTERNAL_BUFFER_H__
#define __LIBCAMERA_INTERNAL_BUFFER_H__

#include <sys/mman.h>
#include <vector>

#include <libcamera/class.h>
#include <libcamera/buffer.h>
#include <libcamera/span.h>

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

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_BUFFER_H__ */
