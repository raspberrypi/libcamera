/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_hal_manager.cpp - libcamera Android Camera Manager
 */

#include "camera_hal_manager.h"

#include <libcamera/camera.h>

#include "log.h"

#include "camera_device.h"
#include "camera_proxy.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL);

/*
 * \class CameraHalManager
 *
 * The HAL camera manager is initializated at camera_module_t 'hal_init()' time
 * and spawns its own thread where libcamera related events are dispatched to.
 * It wraps the libcamera CameraManager operations and provides helpers for the
 * camera_module_t operations, to retrieve the number of cameras in the system,
 * their static information and to open and close camera devices.
 */

int CameraHalManager::init()
{
	/*
	 * Start the camera HAL manager thread and wait until its
	 * initialisation completes to be fully operational before
	 * receiving calls from the camera stack.
	 */
	start();

	MutexLocker locker(mutex_);
	cv_.wait(locker);

	return 0;
}

void CameraHalManager::run()
{
	/*
	 * All the libcamera components must be initialised here, in
	 * order to bind them to the camera HAL manager thread that
	 * executes the event dispatcher.
	 */
	cameraManager_ = libcamera::CameraManager::instance();

	int ret = cameraManager_->start();
	if (ret) {
		LOG(HAL, Error) << "Failed to start camera manager: "
				<< strerror(-ret);
		return;
	}

	/*
	 * For each Camera registered in the system, a CameraProxy
	 * gets created here to wraps a camera device.
	 *
	 * \todo Support camera hotplug.
	 */
	unsigned int index = 0;
	for (auto &camera : cameraManager_->cameras()) {
		CameraProxy *proxy = new CameraProxy(index, camera);
		proxies_.emplace_back(proxy);

		++index;
	}

	/*
	 * libcamera has been initialized. Unlock the init() caller
	 * as we're now ready to handle calls from the framework.
	 */
	cv_.notify_one();

	/* Now start processing events and messages. */
	exec();
}

CameraProxy *CameraHalManager::open(unsigned int id,
				    const hw_module_t *hardwareModule)
{
	if (id >= numCameras()) {
		LOG(HAL, Error) << "Invalid camera id '" << id << "'";
		return nullptr;
	}

	CameraProxy *proxy = proxies_[id].get();
	if (proxy->open(hardwareModule))
		return nullptr;

	LOG(HAL, Info) << "Open camera '" << id << "'";

	return proxy;
}

int CameraHalManager::close(CameraProxy *proxy)
{
	proxy->close();
	LOG(HAL, Info) << "Close camera '" << proxy->id() << "'";

	return 0;
}

unsigned int CameraHalManager::numCameras() const
{
	return cameraManager_->cameras().size();
}

int CameraHalManager::getCameraInfo(unsigned int id, struct camera_info *info)
{
	if (!info)
		return -EINVAL;

	if (id >= numCameras()) {
		LOG(HAL, Error) << "Invalid camera id '" << id << "'";
		return -EINVAL;
	}

	CameraProxy *proxy = proxies_[id].get();

	/* \todo Get these info dynamically inspecting the camera module. */
	info->facing = id ? CAMERA_FACING_FRONT : CAMERA_FACING_BACK;
	info->orientation = 0;
	info->device_version = 0;
	info->resource_cost = 0;
	info->static_camera_characteristics = proxy->getStaticMetadata();
	info->conflicting_devices = nullptr;
	info->conflicting_devices_length = 0;

	return 0;
}
