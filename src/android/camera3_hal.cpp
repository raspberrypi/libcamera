/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera3_hal.cpp - Android Camera HALv3 module
 */

#include <hardware/camera_common.h>

#include "libcamera/internal/log.h"

#include "camera_device.h"
#include "camera_hal_manager.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(HAL)

static CameraHalManager cameraManager;

/*------------------------------------------------------------------------------
 * Android Camera HAL callbacks
 */

static int hal_get_number_of_cameras(void)
{
	return cameraManager.numCameras();
}

static int hal_get_camera_info(int id, struct camera_info *info)
{
	return cameraManager.getCameraInfo(id, info);
}

static int hal_set_callbacks(const camera_module_callbacks_t *callbacks)
{
	return 0;
}

static int hal_open_legacy(const struct hw_module_t *module, const char *id,
			   uint32_t halVersion, struct hw_device_t **device)
{
	return -ENOSYS;
}

static int hal_set_torch_mode(const char *camera_id, bool enabled)
{
	return -ENOSYS;
}

/*
 * First entry point of the camera HAL module.
 *
 * Initialize the HAL but does not open any camera device yet (see hal_dev_open)
 */
static int hal_init()
{
	LOG(HAL, Info) << "Initialising Android camera HAL";

	cameraManager.init();

	return 0;
}

/*------------------------------------------------------------------------------
 * Android Camera Device
 */

static int hal_dev_open(const hw_module_t *module, const char *name,
			hw_device_t **device)
{
	LOG(HAL, Debug) << "Open camera " << name;

	int id = atoi(name);
	CameraDevice *camera = cameraManager.open(id, module);
	if (!camera) {
		LOG(HAL, Error)
			<< "Failed to open camera module '" << id << "'";
		return -ENODEV;
	}

	*device = &camera->camera3Device()->common;

	return 0;
}

static struct hw_module_methods_t hal_module_methods = {
	.open = hal_dev_open,
};

camera_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.module_api_version = CAMERA_MODULE_API_VERSION_2_4,
		.hal_api_version = HARDWARE_HAL_API_VERSION,
		.id = CAMERA_HARDWARE_MODULE_ID,
		.name = "libcamera camera HALv3 module",
		.author = "libcamera",
		.methods = &hal_module_methods,
		.dso = nullptr,
		.reserved = {},
	},

	.get_number_of_cameras = hal_get_number_of_cameras,
	.get_camera_info = hal_get_camera_info,
	.set_callbacks = hal_set_callbacks,
	.get_vendor_tag_ops = nullptr,
	.open_legacy = hal_open_legacy,
	.set_torch_mode = hal_set_torch_mode,
	.init = hal_init,
	.reserved = {},
};
