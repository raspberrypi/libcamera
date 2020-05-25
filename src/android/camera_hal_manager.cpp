/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_hal_manager.cpp - libcamera Android Camera Manager
 */

#include "camera_hal_manager.h"

#include <libcamera/camera.h>

#include "libcamera/internal/log.h"

#include "camera_device.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL);

/*
 * \class CameraHalManager
 *
 * The HAL camera manager is initializated at camera_module_t 'hal_init()' time
 * and spawns its own thread where libcamera related events are dispatched to.
 * It wraps the libcamera CameraManager operations and provides helpers for the
 * camera_module_t operations, to retrieve the number of cameras in the system,
 * their static information and to open camera devices.
 */

CameraHalManager::CameraHalManager()
	: cameraManager_(nullptr)
{
}

CameraHalManager::~CameraHalManager()
{
	cameras_.clear();

	if (cameraManager_) {
		cameraManager_->stop();
		delete cameraManager_;
		cameraManager_ = nullptr;
	}
}

int CameraHalManager::init()
{
	cameraManager_ = new CameraManager();

	int ret = cameraManager_->start();
	if (ret) {
		LOG(HAL, Error) << "Failed to start camera manager: "
				<< strerror(-ret);
		delete cameraManager_;
		cameraManager_ = nullptr;
		return ret;
	}

	/*
	 * For each Camera registered in the system, a CameraDevice
	 * gets created here to wraps a libcamera Camera instance.
	 *
	 * \todo Support camera hotplug.
	 */
	unsigned int index = 0;
	for (auto &cam : cameraManager_->cameras()) {
		CameraDevice *camera = new CameraDevice(index, cam);
		ret = camera->initialize();
		if (ret)
			continue;

		cameras_.emplace_back(camera);
		++index;
	}

	return 0;
}

CameraDevice *CameraHalManager::open(unsigned int id,
				     const hw_module_t *hardwareModule)
{
	if (id >= numCameras()) {
		LOG(HAL, Error) << "Invalid camera id '" << id << "'";
		return nullptr;
	}

	CameraDevice *camera = cameras_[id].get();
	if (camera->open(hardwareModule))
		return nullptr;

	LOG(HAL, Info) << "Open camera '" << id << "'";

	return camera;
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

	CameraDevice *camera = cameras_[id].get();

	info->facing = camera->facing();
	info->orientation = camera->orientation();
	info->device_version = CAMERA_DEVICE_API_VERSION_3_3;
	info->resource_cost = 0;
	info->static_camera_characteristics = camera->getStaticMetadata();
	info->conflicting_devices = nullptr;
	info->conflicting_devices_length = 0;

	return 0;
}
