/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include "py_camera_manager.h"

#include <errno.h>
#include <memory>
#include <sys/eventfd.h>
#include <system_error>
#include <unistd.h>
#include <vector>

#include "py_main.h"

namespace py = pybind11;

using namespace libcamera;

PyCameraManager::PyCameraManager()
{
	LOG(Python, Debug) << "PyCameraManager()";

	cameraManager_ = std::make_unique<CameraManager>();

	int fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (fd == -1)
		throw std::system_error(errno, std::generic_category(),
					"Failed to create eventfd");

	eventFd_ = UniqueFD(fd);

	int ret = cameraManager_->start();
	if (ret)
		throw std::system_error(-ret, std::generic_category(),
					"Failed to start CameraManager");
}

PyCameraManager::~PyCameraManager()
{
	LOG(Python, Debug) << "~PyCameraManager()";
}

py::list PyCameraManager::cameras()
{
	/*
	 * Create a list of Cameras, where each camera has a keep-alive to
	 * CameraManager.
	 */
	py::list l;

	for (auto &camera : cameraManager_->cameras()) {
		py::object py_cm = py::cast(this);
		py::object py_cam = py::cast(camera);
		py::detail::keep_alive_impl(py_cam, py_cm);
		l.append(py_cam);
	}

	return l;
}

std::vector<py::object> PyCameraManager::getReadyRequests()
{
	int ret = readFd();

	if (ret == -EAGAIN)
		return std::vector<py::object>();

	if (ret != 0)
		throw std::system_error(-ret, std::generic_category());

	std::vector<py::object> py_reqs;

	for (Request *request : getCompletedRequests()) {
		py::object o = py::cast(request);
		/* Decrease the ref increased in Camera.queue_request() */
		o.dec_ref();
		py_reqs.push_back(o);
	}

	return py_reqs;
}

/* Note: Called from another thread */
void PyCameraManager::handleRequestCompleted(Request *req)
{
	pushRequest(req);
	writeFd();
}

void PyCameraManager::writeFd()
{
	uint64_t v = 1;

	size_t s = write(eventFd_.get(), &v, 8);
	/*
	 * We should never fail, and have no simple means to manage the error,
	 * so let's log a fatal error.
	 */
	if (s != 8)
		LOG(Python, Fatal) << "Unable to write to eventfd";
}

int PyCameraManager::readFd()
{
	uint8_t buf[8];

	ssize_t ret = read(eventFd_.get(), buf, 8);

	if (ret == 8)
		return 0;
	else if (ret < 0)
		return -errno;
	else
		return -EIO;
}

void PyCameraManager::pushRequest(Request *req)
{
	MutexLocker guard(completedRequestsMutex_);
	completedRequests_.push_back(req);
}

std::vector<Request *> PyCameraManager::getCompletedRequests()
{
	std::vector<Request *> v;
	MutexLocker guard(completedRequestsMutex_);
	swap(v, completedRequests_);
	return v;
}
