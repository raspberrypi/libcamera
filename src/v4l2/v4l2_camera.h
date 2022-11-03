/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_camera.h - V4L2 compatibility camera
 */

#pragma once

#include <deque>
#include <utility>

#include <libcamera/base/mutex.h>
#include <libcamera/base/semaphore.h>
#include <libcamera/base/shared_fd.h>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>

class V4L2Camera
{
public:
	struct Buffer {
		Buffer(unsigned int index, const libcamera::FrameMetadata &data)
			: index_(index), data_(data)
		{
		}

		unsigned int index_;
		libcamera::FrameMetadata data_;
	};

	V4L2Camera(std::shared_ptr<libcamera::Camera> camera);
	~V4L2Camera();

	int open(libcamera::StreamConfiguration *streamConfig);
	void close();
	void bind(int efd);
	void unbind();

	std::vector<Buffer> completedBuffers() LIBCAMERA_TSA_EXCLUDES(bufferLock_);

	int configure(libcamera::StreamConfiguration *streamConfigOut,
		      const libcamera::Size &size,
		      const libcamera::PixelFormat &pixelformat,
		      unsigned int bufferCount);
	int validateConfiguration(const libcamera::PixelFormat &pixelformat,
				  const libcamera::Size &size,
				  libcamera::StreamConfiguration *streamConfigOut);

	int allocBuffers(unsigned int count);
	void freeBuffers();
	int getBufferFd(unsigned int index);

	int streamOn();
	int streamOff();

	int qbuf(unsigned int index);

	void waitForBufferAvailable() LIBCAMERA_TSA_EXCLUDES(bufferMutex_);
	bool isBufferAvailable() LIBCAMERA_TSA_EXCLUDES(bufferMutex_);

	bool isRunning();

private:
	void requestComplete(libcamera::Request *request)
		LIBCAMERA_TSA_EXCLUDES(bufferLock_);

	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;

	bool isRunning_;

	libcamera::Mutex bufferLock_;
	libcamera::FrameBufferAllocator *bufferAllocator_;

	std::vector<std::unique_ptr<libcamera::Request>> requestPool_;

	std::deque<libcamera::Request *> pendingRequests_;
	std::deque<std::unique_ptr<Buffer>> completedBuffers_
		LIBCAMERA_TSA_GUARDED_BY(bufferLock_);

	int efd_;

	libcamera::Mutex bufferMutex_;
	libcamera::ConditionVariable bufferCV_;
	unsigned int bufferAvailableCount_ LIBCAMERA_TSA_GUARDED_BY(bufferMutex_);
};
