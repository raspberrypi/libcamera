/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_worker.h - Process capture requests on behalf of the Camera HAL
 */
#ifndef __ANDROID_CAMERA_WORKER_H__
#define __ANDROID_CAMERA_WORKER_H__

#include <memory>

#include <libcamera/buffer.h>
#include <libcamera/camera.h>
#include <libcamera/object.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/thread.h"

class CameraDevice;

class CaptureRequest
{
public:
	CaptureRequest(libcamera::Camera *camera, uint64_t cookie);

	const std::vector<int> &fences() const { return acquireFences_; }
	libcamera::ControlList &controls() { return request_->controls(); }
	const libcamera::ControlList &metadata() const
	{
		return request_->metadata();
	}
	unsigned long cookie() const { return request_->cookie(); }

	void addBuffer(libcamera::Stream *stream,
		       libcamera::FrameBuffer *buffer, int fence);
	void queue();

private:
	libcamera::Camera *camera_;
	std::vector<int> acquireFences_;
	std::unique_ptr<libcamera::Request> request_;
};

class CameraWorker
{
public:
	CameraWorker();

	void start();
	void stop();

	void queueRequest(CaptureRequest *request);

private:
	class Worker : public libcamera::Object
	{
	public:
		void processRequest(CaptureRequest *request);

	private:
		int waitFence(int fence);
	};

	Worker worker_;
	libcamera::Thread thread_;
};

#endif /* __ANDROID_CAMERA_WORKER_H__ */
