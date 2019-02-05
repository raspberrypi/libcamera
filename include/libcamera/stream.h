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

class Stream final
{
public:
	Stream();
	BufferPool &bufferPool() { return bufferPool_; }

private:
	BufferPool bufferPool_;
};

struct StreamConfiguration {
	unsigned int width;
	unsigned int height;
	unsigned int pixelFormat;

	unsigned int bufferCount;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_STREAM_H__ */
