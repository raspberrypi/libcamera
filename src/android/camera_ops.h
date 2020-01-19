/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_ops.h - Android Camera HAL Operations
 */
#ifndef __ANDROID_CAMERA_OPS_H__
#define __ANDROID_CAMERA_OPS_H__

#include <hardware/camera3.h>

int hal_dev_close(hw_device_t *hw_device);
extern camera3_device_ops hal_dev_ops;

#endif /* __ANDROID_CAMERA_OPS_H__ */
