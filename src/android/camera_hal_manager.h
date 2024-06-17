/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Android Camera Manager
 */

#pragma once

#include <map>
#include <stddef.h>
#include <tuple>
#include <vector>

#include <hardware/camera_common.h>
#include <hardware/hardware.h>
#include <system/camera_metadata.h>

#include <libcamera/base/class.h>
#include <libcamera/base/mutex.h>

#include <libcamera/camera_manager.h>

#include "camera_hal_config.h"

class CameraDevice;

class CameraHalManager
{
public:
	~CameraHalManager();

	static CameraHalManager *instance();

	int init();

	std::tuple<CameraDevice *, int>
	open(unsigned int id, const hw_module_t *module);

	unsigned int numCameras() const;
	int getCameraInfo(unsigned int id, struct camera_info *info);
	void setCallbacks(const camera_module_callbacks_t *callbacks);

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraHalManager)

	static constexpr unsigned int firstExternalCameraId_ = 1000;

	CameraHalManager();

	static int32_t cameraLocation(const libcamera::Camera *cam);

	void cameraAdded(std::shared_ptr<libcamera::Camera> cam);
	void cameraRemoved(std::shared_ptr<libcamera::Camera> cam);

	CameraDevice *cameraDeviceFromHalId(unsigned int id) LIBCAMERA_TSA_REQUIRES(mutex_);

	std::unique_ptr<libcamera::CameraManager> cameraManager_;
	CameraHalConfig halConfig_;

	const camera_module_callbacks_t *callbacks_;
	std::vector<std::unique_ptr<CameraDevice>> cameras_ LIBCAMERA_TSA_GUARDED_BY(mutex_);
	std::map<std::string, unsigned int> cameraIdsMap_ LIBCAMERA_TSA_GUARDED_BY(mutex_);
	libcamera::Mutex mutex_;

	unsigned int numInternalCameras_;
	unsigned int nextExternalCameraId_;
};
