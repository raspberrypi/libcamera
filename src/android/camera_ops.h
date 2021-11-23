/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_ops.h - Android Camera HAL Operations
 */

#pragma once

#include <hardware/camera3.h>

int hal_dev_close(hw_device_t *hw_device);
extern camera3_device_ops hal_dev_ops;
