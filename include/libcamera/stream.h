/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.h - Video stream for a Camera
 */
#ifndef __LIBCAMERA_STREAM_H__
#define __LIBCAMERA_STREAM_H__

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <libcamera/buffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixelformats.h>

namespace libcamera {

class Camera;
class Stream;

class StreamFormats
{
public:
	StreamFormats();
	StreamFormats(const std::map<PixelFormat, std::vector<SizeRange>> &formats);

	std::vector<PixelFormat> pixelformats() const;
	std::vector<Size> sizes(PixelFormat pixelformat) const;

	SizeRange range(PixelFormat pixelformat) const;

private:
	std::map<PixelFormat, std::vector<SizeRange>> formats_;
};

enum MemoryType {
	InternalMemory,
	ExternalMemory,
};

struct StreamConfiguration {
	StreamConfiguration();
	StreamConfiguration(const StreamFormats &formats);

	PixelFormat pixelFormat;
	Size size;

	MemoryType memoryType;
	unsigned int bufferCount;

	Stream *stream() const { return stream_; }
	void setStream(Stream *stream) { stream_ = stream; }
	const StreamFormats &formats() const { return formats_; }

	std::string toString() const;

private:
	Stream *stream_;
	StreamFormats formats_;
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

	std::unique_ptr<Buffer> createBuffer(unsigned int index);
	std::unique_ptr<Buffer> createBuffer(const std::array<int, 3> &fds);

	BufferPool &bufferPool() { return bufferPool_; }
	std::vector<BufferMemory> &buffers() { return bufferPool_.buffers(); }
	const StreamConfiguration &configuration() const { return configuration_; }
	MemoryType memoryType() const { return memoryType_; }

protected:
	friend class Camera;

	int mapBuffer(const Buffer *buffer);
	void unmapBuffer(const Buffer *buffer);

	void createBuffers(MemoryType memory, unsigned int count);
	void destroyBuffers();

	BufferPool bufferPool_;
	StreamConfiguration configuration_;
	MemoryType memoryType_;

private:
	std::vector<std::pair<std::array<int, 3>, unsigned int>> bufferCache_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_STREAM_H__ */
