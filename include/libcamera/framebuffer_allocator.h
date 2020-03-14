/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * framebuffer_allocator.h - FrameBuffer allocator
 */
#ifndef __LIBCAMERA_FRAMEBUFFER_ALLOCATOR_H__
#define __LIBCAMERA_FRAMEBUFFER_ALLOCATOR_H__

#include <map>
#include <memory>
#include <vector>

namespace libcamera {

class Camera;
class FrameBuffer;
class Stream;

class FrameBufferAllocator
{
public:
	FrameBufferAllocator(std::shared_ptr<Camera> camera);
	FrameBufferAllocator(const Camera &) = delete;
	FrameBufferAllocator &operator=(const Camera &) = delete;

	~FrameBufferAllocator();

	int allocate(Stream *stream);
	int free(Stream *stream);

	bool allocated() const { return !buffers_.empty(); }
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers(Stream *stream) const;

private:
	std::shared_ptr<Camera> camera_;
	std::map<Stream *, std::vector<std::unique_ptr<FrameBuffer>>> buffers_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_FRAMEBUFFER_ALLOCATOR_H__ */
