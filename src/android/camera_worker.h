/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_worker.h - Process capture requests on behalf of the Camera HAL
 */
#ifndef __ANDROID_CAMERA_WORKER_H__
#define __ANDROID_CAMERA_WORKER_H__

#include <memory>

#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

class CameraDevice;

class CaptureRequest
{
public:
	CaptureRequest(libcamera::Camera *camera);

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

class CameraWorker : private libcamera::Thread
{
public:
	CameraWorker();

	void start();
	void stop();

	void queueRequest(CaptureRequest *request);

protected:
	void run() override;

private:
	class Worker : public libcamera::Object
	{
	public:
		void processRequest(CaptureRequest *request);

	private:
		int waitFence(int fence);
	};

	Worker worker_;
};

#endif /* __ANDROID_CAMERA_WORKER_H__ */
