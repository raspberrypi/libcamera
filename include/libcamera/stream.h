/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.h - Video stream for a Camera
 */
#ifndef __LIBCAMERA_STREAM_H__
#define __LIBCAMERA_STREAM_H__

#include <string>
#include <vector>

#include <libcamera/buffer.h>
#include <libcamera/geometry.h>

namespace libcamera {

class Camera;
class Stream;

struct StreamConfiguration {
	StreamConfiguration()
		: stream_(nullptr)
	{
	}

	unsigned int pixelFormat;
	Size size;

	unsigned int bufferCount;

	Stream *stream() const { return stream_; }
	void setStream(Stream *stream) { stream_ = stream; }

	std::string toString() const;

private:
	Stream *stream_;
};

enum StreamRole {
	StillCapture,
	VideoRecording,
	Viewfinder,
};

using StreamRoles = std::vector<StreamRole>;

class Stream
{
public:
	Stream();
	BufferPool &bufferPool() { return bufferPool_; }
	const StreamConfiguration &configuration() const { return configuration_; }

protected:
	friend class Camera;

	BufferPool bufferPool_;
	StreamConfiguration configuration_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_STREAM_H__ */
