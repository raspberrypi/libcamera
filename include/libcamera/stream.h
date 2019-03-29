/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.h - Video stream for a Camera
 */
#ifndef __LIBCAMERA_STREAM_H__
#define __LIBCAMERA_STREAM_H__

#include <libcamera/buffer.h>

namespace libcamera {

class Camera;

struct StreamConfiguration {
	unsigned int width;
	unsigned int height;
	unsigned int pixelFormat;

	unsigned int bufferCount;
};

class Stream final
{
public:
	Stream();
	BufferPool &bufferPool() { return bufferPool_; }
	const StreamConfiguration &configuration() const { return configuration_; }

private:
	friend class Camera;

	BufferPool bufferPool_;
	StreamConfiguration configuration_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_STREAM_H__ */
