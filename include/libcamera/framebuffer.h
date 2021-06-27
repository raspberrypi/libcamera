/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * framebuffer.h - Frame buffer handling
 */
#ifndef __LIBCAMERA_FRAMEBUFFER_H__
#define __LIBCAMERA_FRAMEBUFFER_H__

#include <stdint.h>
#include <vector>

#include <libcamera/base/class.h>

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

class FrameBuffer final : public Extensible
{
	LIBCAMERA_DECLARE_PRIVATE()

public:
	struct Plane {
		FileDescriptor fd;
		unsigned int length;
	};

	FrameBuffer(const std::vector<Plane> &planes, unsigned int cookie = 0);

	const std::vector<Plane> &planes() const { return planes_; }

	Request *request() const;
	const FrameMetadata &metadata() const { return metadata_; }

	unsigned int cookie() const { return cookie_; }
	void setCookie(unsigned int cookie) { cookie_ = cookie; }

	void cancel() { metadata_.status = FrameMetadata::FrameCancelled; }

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(FrameBuffer)

	friend class V4L2VideoDevice; /* Needed to update metadata_. */

	std::vector<Plane> planes_;

	FrameMetadata metadata_;

	unsigned int cookie_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_FRAMEBUFFER_H__ */
