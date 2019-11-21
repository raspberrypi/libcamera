/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_camera.h - V4L2 compatibility camera
 */

#ifndef __V4L2_CAMERA_H__
#define __V4L2_CAMERA_H__

#include <deque>
#include <linux/videodev2.h>
#include <mutex>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/file_descriptor.h>

#include "semaphore.h"

using namespace libcamera;

class V4L2FrameMetadata
{
public:
	V4L2FrameMetadata(Buffer *buffer);

	int index() const { return index_; }

	unsigned int bytesused() const { return bytesused_; }
	uint64_t timestamp() const { return timestamp_; }
	unsigned int sequence() const { return sequence_; }

	FrameMetadata::Status status() const { return status_; }

private:
	int index_;

	unsigned int bytesused_;
	uint64_t timestamp_;
	unsigned int sequence_;

	FrameMetadata::Status status_;
};

class V4L2Camera : public Object
{
public:
	V4L2Camera(std::shared_ptr<Camera> camera);
	~V4L2Camera();

	int open();
	void close();
	void getStreamConfig(StreamConfiguration *streamConfig);
	std::vector<V4L2FrameMetadata> completedBuffers();

	int configure(StreamConfiguration *streamConfigOut,
		      const Size &size, PixelFormat pixelformat,
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

	std::deque<std::unique_ptr<Request>> pendingRequests_;
	std::deque<std::unique_ptr<V4L2FrameMetadata>> completedBuffers_;
};

#endif /* __V4L2_CAMERA_H__ */
