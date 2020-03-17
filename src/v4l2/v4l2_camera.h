/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_camera.h - V4L2 compatibility camera
 */

#ifndef __V4L2_CAMERA_H__
#define __V4L2_CAMERA_H__

#include <deque>
#include <mutex>
#include <utility>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/file_descriptor.h>
#include <libcamera/framebuffer_allocator.h>

#include "semaphore.h"

using namespace libcamera;

class V4L2Camera
{
public:
	struct Buffer {
		Buffer(unsigned int index, const FrameMetadata &data)
			: index(index), data(data)
		{
		}

		unsigned int index;
		FrameMetadata data;
	};

	V4L2Camera(std::shared_ptr<Camera> camera);
	~V4L2Camera();

	int open();
	void close();
	void getStreamConfig(StreamConfiguration *streamConfig);
	std::vector<Buffer> completedBuffers();

	int configure(StreamConfiguration *streamConfigOut,
		      const Size &size, const PixelFormat &pixelformat,
		      unsigned int bufferCount);

	int allocBuffers(unsigned int count);
	void freeBuffers();
	FileDescriptor getBufferFd(unsigned int index);

	int streamOn();
	int streamOff();

	int qbuf(unsigned int index);

	Semaphore bufferSema_;

private:
	void requestComplete(Request *request);

	std::shared_ptr<Camera> camera_;
	std::unique_ptr<CameraConfiguration> config_;

	bool isRunning_;

	std::mutex bufferLock_;
	FrameBufferAllocator *bufferAllocator_;

	std::deque<std::unique_ptr<Request>> pendingRequests_;
	std::deque<std::unique_ptr<Buffer>> completedBuffers_;
};

#endif /* __V4L2_CAMERA_H__ */
