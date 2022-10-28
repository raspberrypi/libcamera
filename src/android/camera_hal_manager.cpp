/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_hal_manager.cpp - libcamera Android Camera Manager
 */

#include "camera_hal_manager.h"

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/property_ids.h>

#include "camera_device.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL)

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

/* CameraManager calls stop() in the destructor. */
CameraHalManager::~CameraHalManager() = default;

/* static */
CameraHalManager *CameraHalManager::instance()
{
	static CameraHalManager *cameraHalManager = new CameraHalManager;
	return cameraHalManager;
}

int CameraHalManager::init()
{
	cameraManager_ = std::make_unique<CameraManager>();

	/*
	 * If the configuration file is not available the HAL only supports
	 * external cameras. If it exists but it's not valid then error out.
	 */
	if (halConfig_.exists() && !halConfig_.isValid()) {
		LOG(HAL, Error) << "HAL configuration file is not valid";
		return -EINVAL;
	}

	/* Support camera hotplug. */
	cameraManager_->cameraAdded.connect(this, &CameraHalManager::cameraAdded);
	cameraManager_->cameraRemoved.connect(this, &CameraHalManager::cameraRemoved);

	int ret = cameraManager_->start();
	if (ret) {
		LOG(HAL, Error) << "Failed to start camera manager: "
				<< strerror(-ret);
		cameraManager_.reset();
		return ret;
	}

	return 0;
}

std::tuple<CameraDevice *, int>
CameraHalManager::open(unsigned int id, const hw_module_t *hardwareModule)
{
	MutexLocker locker(mutex_);

	if (!callbacks_) {
		LOG(HAL, Error) << "Can't open camera before callbacks are set";
		return { nullptr, -ENODEV };
	}

	CameraDevice *camera = cameraDeviceFromHalId(id);
	if (!camera) {
		LOG(HAL, Error) << "Invalid camera id '" << id << "'";
		return { nullptr, -ENODEV };
	}

	int ret = camera->open(hardwareModule);
	if (ret)
		return { nullptr, ret };

	LOG(HAL, Info) << "Open camera '" << id << "'";

	return { camera, 0 };
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
		if (id >= firstExternalCameraId_)
			isCameraExternal = true;
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

	/*
	 * The configuration file must be valid, and contain a corresponding
	 * entry for internal cameras. External cameras can be initialized
	 * without configuration file.
	 */
	if (!isCameraExternal && !halConfig_.exists()) {
		LOG(HAL, Error)
			<< "HAL configuration file is mandatory for internal cameras."
			<< " Camera " << cam->id() << "failed to load";
		return;
	}

	const CameraConfigData *cameraConfigData = halConfig_.cameraConfigData(cam->id());

	/*
	 * Some cameras whose location is reported by libcamera as external may
	 * actually be internal to the device. This is common with UVC cameras
	 * that are integrated in a laptop. In that case the real location
	 * should be specified in the configuration file.
	 *
	 * If the camera location is external and a configuration entry exists
	 * for it, override its location.
	 */
	if (isCameraNew && isCameraExternal) {
		if (cameraConfigData && cameraConfigData->facing != -1) {
			isCameraExternal = false;
			id = numInternalCameras_;
		}
	}

	if (!isCameraExternal && !cameraConfigData) {
		LOG(HAL, Error)
			<< "HAL configuration entry for internal camera "
			<< cam->id() << " is missing";
		return;
	}

	/* Create a CameraDevice instance to wrap the libcamera Camera. */
	std::unique_ptr<CameraDevice> camera = CameraDevice::create(id, cam);

	int ret = camera->initialize(cameraConfigData);
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
				 [&cam](const std::unique_ptr<CameraDevice> &camera) {
					 return cam == camera->camera();
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
	return cam->properties().get(properties::Location).value_or(-1);
}

CameraDevice *CameraHalManager::cameraDeviceFromHalId(unsigned int id)
{
	auto iter = std::find_if(cameras_.begin(), cameras_.end(),
				 [id](const std::unique_ptr<CameraDevice> &camera) {
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
	for (const std::unique_ptr<CameraDevice> &camera : cameras_) {
		unsigned int id = camera->id();
		if (id >= firstExternalCameraId_)
			callbacks_->camera_device_status_change(callbacks_, id,
								CAMERA_DEVICE_STATUS_PRESENT);
	}
}
