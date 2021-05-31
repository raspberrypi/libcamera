/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera3_hal.cpp - cros-specific components of Android Camera HALv3 module
 */

#include <cros-camera/cros_camera_hal.h>

#include "../camera_hal_manager.h"

static void set_up([[maybe_unused]] cros::CameraMojoChannelManagerToken *token)
{
}

static void tear_down()
{
	delete CameraHalManager::instance();
}

cros::cros_camera_hal_t CROS_CAMERA_EXPORT CROS_CAMERA_HAL_INFO_SYM = {
	.set_up = set_up,
	.tear_down = tear_down
};
