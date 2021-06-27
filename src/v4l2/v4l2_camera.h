/* SPDX-License-Identifier: LGPL-2.1-or-later */
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

#include <libcamera/base/semaphore.h>

#include <libcamera/camera.h>
#include <libcamera/file_descriptor.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>

using namespace libcamera;

class V4L2Camera
{
public:
	struct Buffer {
		Buffer(unsigned int index, const FrameMetadata &data)
			: index_(index), data_(data)
		{
		}

		unsigned int index_;
		FrameMetadata data_;
	};

	V4L2Camera(std::shared_ptr<Camera> camera);
	~V4L2Camera();

	int open(StreamConfiguration *streamConfig);
	void close();
	void bind(int efd);
	void unbind();

	std::vector<Buffer> completedBuffers();

	int configure(StreamConfiguration *streamConfigOut,
		      const Size &size, const PixelFormat &pixelformat,
		      unsigned int bufferCount);
	int validateConfiguration(const PixelFormat &pixelformat,
				  const Size &size,
				  StreamConfiguration *streamConfigOut);

	int allocBuffers(unsigned int count);
	void freeBuffers();
	FileDescriptor getBufferFd(unsigned int index);

	int streamOn();
	int streamOff();

	int qbuf(unsigned int index);

	void waitForBufferAvailable();
	bool isBufferAvailable();

	bool isRunning();

private:
	void requestComplete(Request *request);

	std::shared_ptr<Camera> camera_;
	std::unique_ptr<CameraConfiguration> config_;

	bool isRunning_;

	std::mutex bufferLock_;
	FrameBufferAllocator *bufferAllocator_;

	std::vector<std::unique_ptr<Request>> requestPool_;

	std::deque<Request *> pendingRequests_;
	std::deque<std::unique_ptr<Buffer>> completedBuffers_;

	int efd_;

	Mutex bufferMutex_;
	std::condition_variable bufferCV_;
	unsigned int bufferAvailableCount_;
};

#endif /* __V4L2_CAMERA_H__ */
