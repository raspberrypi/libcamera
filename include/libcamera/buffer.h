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

#include <libcamera/file_descriptor.h>

namespace libcamera {

class Request;
class Stream;

struct FrameMetadata {
	enum Status {
		FrameSuccess,
		FrameError,
		FrameCancelled,
	};

	struct Plane {
		unsigned int bytesused;
	};

	Status status;
	unsigned int sequence;
	uint64_t timestamp;
	std::vector<Plane> planes;
};

class FrameBuffer final
{
public:
	struct Plane {
		FileDescriptor fd;
		unsigned int length;
	};

	FrameBuffer(const std::vector<Plane> &planes, unsigned int cookie = 0);

	FrameBuffer(const FrameBuffer &) = delete;
	FrameBuffer(FrameBuffer &&) = delete;

	FrameBuffer &operator=(const FrameBuffer &) = delete;
	FrameBuffer &operator=(FrameBuffer &&) = delete;

	const std::vector<Plane> &planes() const { return planes_; }

	Request *request() const { return request_; }
	const FrameMetadata &metadata() const { return metadata_; };

	unsigned int cookie() const { return cookie_; }
	void setCookie(unsigned int cookie) { cookie_ = cookie; }

private:
	friend class Request; /* Needed to update request_. */
	friend class V4L2VideoDevice; /* Needed to update metadata_. */

	std::vector<Plane> planes_;

	Request *request_;
	FrameMetadata metadata_;

	unsigned int cookie_;
};

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
	const std::vector<Plane> &planes() const { return planes_; }
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
