/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_hal_manager.cpp - libcamera Android Camera Manager
 */

#include "camera_hal_manager.h"

#include <libcamera/camera.h>
#include <libcamera/property_ids.h>

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
	: cameraManager_(nullptr), callbacks_(nullptr), numInternalCameras_(0),
	  nextExternalCameraId_(firstExternalCameraId_)
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

	/* Support camera hotplug. */
	cameraManager_->cameraAdded.connect(this, &CameraHalManager::cameraAdded);
	cameraManager_->cameraRemoved.connect(this, &CameraHalManager::cameraRemoved);

	int ret = cameraManager_->start();
	if (ret) {
		LOG(HAL, Error) << "Failed to start camera manager: "
				<< strerror(-ret);
		delete cameraManager_;
		cameraManager_ = nullptr;
		return ret;
	}

	return 0;
}

CameraDevice *CameraHalManager::open(unsigned int id,
				     const hw_module_t *hardwareModule)
{
	MutexLocker locker(mutex_);

	if (!callbacks_) {
		LOG(HAL, Error) << "Can't open camera before callbacks are set";
		return nullptr;
	}

	CameraDevice *camera = cameraDeviceFromHalId(id);
	if (!camera) {
		LOG(HAL, Error) << "Invalid camera id '" << id << "'";
		return nullptr;
	}

	if (camera->open(hardwareModule))
		return nullptr;

	LOG(HAL, Info) << "Open camera '" << id << "'";

	return camera;
}

void CameraHalManager::cameraAdded(std::shared_ptr<Camera> cam)
{
	unsigned int id;
	bool isCameraExternal = false;
	bool isCameraNew = false;

	MutexLocker locker(mutex_);

	/*
	 * Each camera is assigned a unique integer ID when it is seen for the
	 * first time. If the camera has been seen before, the previous ID is
	 * re-used.
	 *
	 * IDs starts from '0' for internal cameras and '1000' for external
	 * cameras.
	 */
	auto iter = cameraIdsMap_.find(cam->id());
	if (iter != cameraIdsMap_.end()) {
		id = iter->second;
	} else {
		isCameraNew = true;

		/*
		 * Now check if this is an external camera and assign
		 * its id accordingly.
		 */
		if (cameraLocation(cam.get()) == properties::CameraLocationExternal) {
			isCameraExternal = true;
			id = nextExternalCameraId_;
		} else {
			id = numInternalCameras_;
		}
	}

	/* Create a CameraDevice instance to wrap the libcamera Camera. */
	std::shared_ptr<CameraDevice> camera = CameraDevice::create(id, std::move(cam));
	int ret = camera->initialize();
	if (ret) {
		LOG(HAL, Error) << "Failed to initialize camera: " << cam->id();
		return;
	}

	if (isCameraNew) {
		cameraIdsMap_.emplace(cam->id(), id);

		if (isCameraExternal)
			nextExternalCameraId_++;
		else
			numInternalCameras_++;
	}

	cameras_.emplace_back(std::move(camera));

	if (callbacks_)
		callbacks_->camera_device_status_change(callbacks_, id,
							CAMERA_DEVICE_STATUS_PRESENT);

	LOG(HAL, Debug) << "Camera ID: " << id << " added successfully.";
}

void CameraHalManager::cameraRemoved(std::shared_ptr<Camera> cam)
{
	MutexLocker locker(mutex_);

	auto iter = std::find_if(cameras_.begin(), cameras_.end(),
				 [&cam](std::shared_ptr<CameraDevice> &camera) {
					 return cam.get() == camera->camera();
				 });
	if (iter == cameras_.end())
		return;

	/*
	 * CAMERA_DEVICE_STATUS_NOT_PRESENT should be set for external cameras
	 * only.
	 */
	unsigned int id = (*iter)->id();
	if (id >= firstExternalCameraId_)
		callbacks_->camera_device_status_change(callbacks_, id,
							CAMERA_DEVICE_STATUS_NOT_PRESENT);

	/*
	 * \todo Check if the camera is already open and running.
	 * Inform the framework about its absence before deleting its
	 * reference here.
	 */
	cameras_.erase(iter);

	LOG(HAL, Debug) << "Camera ID: " << id << " removed successfully.";
}

int32_t CameraHalManager::cameraLocation(const Camera *cam)
{
	const ControlList &properties = cam->properties();
	if (!properties.contains(properties::Location))
		return -1;

	return properties.get(properties::Location);
}

CameraDevice *CameraHalManager::cameraDeviceFromHalId(unsigned int id)
{
	auto iter = std::find_if(cameras_.begin(), cameras_.end(),
				 [id](std::shared_ptr<CameraDevice> &camera) {
					 return camera->id() == id;
				 });
	if (iter == cameras_.end())
		return nullptr;

	return iter->get();
}

unsigned int CameraHalManager::numCameras() const
{
	return numInternalCameras_;
}

int CameraHalManager::getCameraInfo(unsigned int id, struct camera_info *info)
{
	if (!info)
		return -EINVAL;

	MutexLocker locker(mutex_);

	CameraDevice *camera = cameraDeviceFromHalId(id);
	if (!camera) {
		LOG(HAL, Error) << "Invalid camera id '" << id << "'";
		return -EINVAL;
	}

	info->facing = camera->facing();
	info->orientation = camera->orientation();
	info->device_version = CAMERA_DEVICE_API_VERSION_3_3;
	info->resource_cost = 0;
	info->static_camera_characteristics = camera->getStaticMetadata();
	info->conflicting_devices = nullptr;
	info->conflicting_devices_length = 0;

	return 0;
}

void CameraHalManager::setCallbacks(const camera_module_callbacks_t *callbacks)
{
	callbacks_ = callbacks;

	MutexLocker locker(mutex_);

	/*
	 * Some external cameras may have been identified before the callbacks_
	 * were set. Iterate all existing external cameras and mark them as
	 * CAMERA_DEVICE_STATUS_PRESENT explicitly.
	 *
	 * Internal cameras are already assumed to be present at module load
	 * time by the Android framework.
	 */
	for (std::shared_ptr<CameraDevice> &camera : cameras_) {
		unsigned int id = camera->id();
		if (id >= firstExternalCameraId_)
			callbacks_->camera_device_status_change(callbacks_, id,
								CAMERA_DEVICE_STATUS_PRESENT);
	}
}
