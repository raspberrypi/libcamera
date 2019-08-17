/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_hal_manager.h - libcamera Android Camera Manager
 */
#ifndef __ANDROID_CAMERA_MANAGER_H__
#define __ANDROID_CAMERA_MANAGER_H__

#include <condition_variable>
#include <mutex>
#include <vector>

#include <hardware/hardware.h>
#include <system/camera_metadata.h>

#include <libcamera/camera_manager.h>

#include "thread.h"

class CameraDevice;
class CameraProxy;

class CameraHalManager : public libcamera::Thread
{
public:
	~CameraHalManager();

	int init();

	CameraProxy *open(unsigned int id, const hw_module_t *module);

	unsigned int numCameras() const;
	int getCameraInfo(unsigned int id, struct camera_info *info);

private:
	void run() override;
	camera_metadata_t *getStaticMetadata(unsigned int id);

	libcamera::CameraManager *cameraManager_;

	std::mutex mutex_;
	std::condition_variable cv_;

	std::vector<std::unique_ptr<CameraProxy>> proxies_;
};

#endif /* __ANDROID_CAMERA_MANAGER_H__ */
