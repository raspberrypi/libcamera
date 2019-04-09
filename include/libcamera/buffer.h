/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * buffer.h - Buffer handling
 */
#ifndef __LIBCAMERA_BUFFER_H__
#define __LIBCAMERA_BUFFER_H__

#include <stdint.h>
#include <vector>

namespace libcamera {

class BufferPool;
class Request;

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

class Buffer final
{
public:
	enum Status {
		BufferSuccess,
		BufferError,
		BufferCancelled,
	};

	Buffer();

	unsigned int index() const { return index_; }
	unsigned int bytesused() const { return bytesused_; }
	uint64_t timestamp() const { return timestamp_; }
	unsigned int sequence() const { return sequence_; }
	Status status() const { return status_; }
	std::vector<Plane> &planes() { return planes_; }
	Request *request() const { return request_; }

private:
	friend class BufferPool;
	friend class PipelineHandler;
	friend class Request;
	friend class V4L2Device;

	void cancel();

	void setRequest(Request *request) { request_ = request; }

	unsigned int index_;
	unsigned int bytesused_;
	uint64_t timestamp_;
	unsigned int sequence_;
	Status status_;

	std::vector<Plane> planes_;
	Request *request_;
};

class BufferPool final
{
public:
	~BufferPool();

	void createBuffers(unsigned int count);
	void destroyBuffers();

	unsigned int count() const { return buffers_.size(); }
	std::vector<Buffer> &buffers() { return buffers_; }

private:
	std::vector<Buffer> buffers_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_BUFFER_H__ */
