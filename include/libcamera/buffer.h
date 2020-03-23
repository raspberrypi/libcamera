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

#include <libcamera/file_descriptor.h>

namespace libcamera {

class Request;

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
	void setRequest(Request *request) { request_ = request; }
	const FrameMetadata &metadata() const { return metadata_; }

	unsigned int cookie() const { return cookie_; }
	void setCookie(unsigned int cookie) { cookie_ = cookie; }

	int copyFrom(const FrameBuffer *src);
private:
	friend class Request; /* Needed to update request_. */
	friend class V4L2VideoDevice; /* Needed to update metadata_. */

	std::vector<Plane> planes_;

	Request *request_;
	FrameMetadata metadata_;

	unsigned int cookie_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_BUFFER_H__ */
