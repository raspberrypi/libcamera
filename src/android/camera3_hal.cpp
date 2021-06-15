/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera3_hal.cpp - Android Camera HALv3 module
 */

#include <hardware/camera_common.h>

#include <libcamera/base/log.h>

#include "camera_device.h"
#include "camera_hal_manager.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(HAL)

/*------------------------------------------------------------------------------
 * Android Camera HAL callbacks
 */

static int hal_get_number_of_cameras()
{
	return CameraHalManager::instance()->numCameras();
}

static int hal_get_camera_info(int id, struct camera_info *info)
{
	return CameraHalManager::instance()->getCameraInfo(id, info);
}

static int hal_set_callbacks(const camera_module_callbacks_t *callbacks)
{
	CameraHalManager::instance()->setCallbacks(callbacks);

	return 0;
}

static int hal_open_legacy([[maybe_unused]] const struct hw_module_t *module,
			   [[maybe_unused]] const char *id,
			   [[maybe_unused]] uint32_t halVersion,
			   [[maybe_unused]] struct hw_device_t **device)
{
	return -ENOSYS;
}

static int hal_set_torch_mode([[maybe_unused]] const char *camera_id,
			      [[maybe_unused]] bool enabled)
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

	CameraHalManager::instance()->init();

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

	auto [camera, ret] = CameraHalManager::instance()->open(id, module);
	if (!camera) {
		LOG(HAL, Error)
			<< "Failed to open camera module '" << id << "'";
		return ret == -EBUSY ? -EUSERS : ret;
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
