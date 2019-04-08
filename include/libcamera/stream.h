/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.h - Video stream for a Camera
 */
#ifndef __LIBCAMERA_STREAM_H__
#define __LIBCAMERA_STREAM_H__

#include <libcamera/buffer.h>
#include <libcamera/geometry.h>

namespace libcamera {

class Camera;

struct StreamConfiguration {
	unsigned int width;
	unsigned int height;
	unsigned int pixelFormat;

	unsigned int bufferCount;
};

class StreamUsage
{
public:
	enum Role {
		StillCapture,
		VideoRecording,
		Viewfinder,
	};

	Role role() const { return role_; }
	const Size &size() const { return size_; }

protected:
	explicit StreamUsage(Role role);
	StreamUsage(Role role, int width, int height);

private:
	Role role_;
	Size size_;
};

class Stream
{
public:
	class StillCapture : public StreamUsage
	{
	public:
		StillCapture();
	};

	class VideoRecording : public StreamUsage
	{
	public:
		VideoRecording();
	};

	class Viewfinder : public StreamUsage
	{
	public:
		Viewfinder(int width, int height);
	};

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
