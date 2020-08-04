/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_hal_manager.h - libcamera Android Camera Manager
 */
#ifndef __ANDROID_CAMERA_MANAGER_H__
#define __ANDROID_CAMERA_MANAGER_H__

#include <stddef.h>
#include <vector>

#include <hardware/hardware.h>
#include <system/camera_metadata.h>

#include <libcamera/camera_manager.h>

class CameraDevice;

class CameraHalManager
{
public:
	CameraHalManager();
	~CameraHalManager();

	int init();

	CameraDevice *open(unsigned int id, const hw_module_t *module);

	unsigned int numCameras() const;
	int getCameraInfo(unsigned int id, struct camera_info *info);

private:
	libcamera::CameraManager *cameraManager_;

	std::vector<std::unique_ptr<CameraDevice>> cameras_;
};

#endif /* __ANDROID_CAMERA_MANAGER_H__ */
