/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_ids.h : Control ID list
 */

#ifndef __LIBCAMERA_CONTROL_IDS_H__
#define __LIBCAMERA_CONTROL_IDS_H__

#include <stdint.h>

#include <libcamera/controls.h>

namespace libcamera {

namespace controls {

enum {
	AWB_ENABLE = 1,
	BRIGHTNESS = 2,
	CONTRAST = 3,
	SATURATION = 4,
	MANUAL_EXPOSURE = 5,
	MANUAL_GAIN = 6,
};

extern const Control<bool> AwbEnable;
extern const Control<int32_t> Brightness;
extern const Control<int32_t> Contrast;
extern const Control<int32_t> Saturation;
extern const Control<int32_t> ManualExposure;
extern const Control<int32_t> ManualGain;

} /* namespace controls */

} /* namespace libcamera */

#endif // __LIBCAMERA_CONTROL_IDS_H__
