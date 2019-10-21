/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer.h - Buffer handling
 */
#ifndef __LIBCAMERA_BUFFER_H__
#define __LIBCAMERA_BUFFER_H__

#include <array>
#include <stdint.h>
#include <vector>

namespace libcamera {

class Request;
class Stream;

class Plane final
{
public:
	Plane();
	~Plane();

	int dmabuf() const { return fd_; }
	int setDmabuf(int fd, unsigned int length);

	void *mem();
	unsigned int length() const { return length_; }

private:
	int mmap();
	int munmap();

	int fd_;
	unsigned int length_;
	void *mem_;
};

class BufferMemory final
{
public:
	std::vector<Plane> &planes() { return planes_; }

private:
	std::vector<Plane> planes_;
};

class BufferPool final
{
public:
	~BufferPool();

	void createBuffers(unsigned int count);
	void destroyBuffers();

	unsigned int count() const { return buffers_.size(); }
	std::vector<BufferMemory> &buffers() { return buffers_; }

private:
	std::vector<BufferMemory> buffers_;
};

class Buffer final
{
public:
	enum Status {
		BufferSuccess,
		BufferError,
		BufferCancelled,
	};

	Buffer(unsigned int index = -1, const Buffer *metadata = nullptr);
	Buffer(const Buffer &) = delete;
	Buffer &operator=(const Buffer &) = delete;

	unsigned int index() const { return index_; }
	const std::array<int, 3> &dmabufs() const { return dmabuf_; }
	BufferMemory *mem() { return mem_; }

	unsigned int bytesused() const { return bytesused_; }
	uint64_t timestamp() const { return timestamp_; }
	unsigned int sequence() const { return sequence_; }

	Status status() const { return status_; }
	Request *request() const { return request_; }
	Stream *stream() const { return stream_; }

private:
	friend class Camera;
	friend class Request;
	friend class Stream;
	friend class V4L2VideoDevice;

	void cancel();

	void setRequest(Request *request) { request_ = request; }

	unsigned int index_;
	std::array<int, 3> dmabuf_;
	BufferMemory *mem_;

	unsigned int bytesused_;
	uint64_t timestamp_;
	unsigned int sequence_;

	Status status_;
	Request *request_;
	Stream *stream_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_BUFFER_H__ */
