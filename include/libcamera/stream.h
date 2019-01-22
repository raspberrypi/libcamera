/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.h - Video stream for a Camera
 */
#ifndef __LIBCAMERA_STREAM_H__
#define __LIBCAMERA_STREAM_H__

namespace libcamera {

class Stream final
{
};

struct StreamConfiguration {
	unsigned int width;
	unsigned int height;
	unsigned int pixelFormat;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_STREAM_H__ */
