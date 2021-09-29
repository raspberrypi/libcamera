/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_worker.cpp - Process capture requests on behalf of the Camera HAL
 */

#include "camera_worker.h"

#include <errno.h>
#include <string.h>
#include <sys/poll.h>
#include <unistd.h>

#include "camera_device.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

/*
 * \class CaptureRequest
 * \brief Wrap a libcamera::Request associated with buffers and fences
 *
 * A CaptureRequest is constructed by the CameraDevice, filled with
 * buffers and fences provided by the camera3 framework and then processed
 * by the CameraWorker which queues it to the libcamera::Camera after handling
 * fences.
 */
CaptureRequest::CaptureRequest(Camera *camera, uint64_t cookie)
	: camera_(camera)
{
	request_ = camera_->createRequest(cookie);
}

void CaptureRequest::addBuffer(Stream *stream, FrameBuffer *buffer, int fence)
{
	request_->addBuffer(stream, buffer);
	acquireFences_.push_back(fence);
}

void CaptureRequest::queue()
{
	camera_->queueRequest(request_.get());
}

/*
 * \class CameraWorker
 * \brief Process a CaptureRequest on an internal thread
 *
 * The CameraWorker class wraps a Worker that runs on an internal thread
 * and schedules processing of CaptureRequest through it.
 */
CameraWorker::CameraWorker()
{
	worker_.moveToThread(this);
}

void CameraWorker::start()
{
	Thread::start();
}

void CameraWorker::stop()
{
	exit();
	wait();
}

void CameraWorker::run()
{
	exec();
	dispatchMessages(Message::Type::InvokeMessage);
}

void CameraWorker::queueRequest(CaptureRequest *request)
{
	/* Async process the request on the worker which runs its own thread. */
	worker_.invokeMethod(&Worker::processRequest, ConnectionTypeQueued,
			     request);
}

/*
 * \class CameraWorker::Worker
 * \brief Process a CaptureRequest handling acquisition fences
 */
int CameraWorker::Worker::waitFence(int fence)
{
	/*
	 * \todo Better characterize the timeout. Currently equal to the one
	 * used by the Rockchip Camera HAL on ChromeOS.
	 */
	constexpr unsigned int timeoutMs = 300;
	struct pollfd fds = { fence, POLLIN, 0 };

	do {
		int ret = poll(&fds, 1, timeoutMs);
		if (ret == 0)
			return -ETIME;

		if (ret > 0) {
			if (fds.revents & (POLLERR | POLLNVAL))
				return -EINVAL;

			return 0;
		}
	} while (errno == EINTR || errno == EAGAIN);

	return -errno;
}

void CameraWorker::Worker::processRequest(CaptureRequest *request)
{
	/* Wait on all fences before queuing the Request. */
	for (int fence : request->fences()) {
		if (fence == -1)
			continue;

		int ret = waitFence(fence);
		close(fence);
		if (ret < 0) {
			LOG(HAL, Error) << "Failed waiting for fence: "
					<< fence << ": " << strerror(-ret);
			return;
		}
	}

	request->queue();
}
