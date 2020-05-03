/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * device_status.h - device (image sensor) status
 */
#pragma once

// Definition of "device metadata" which stores things like shutter time and
// analogue gain that downstream control algorithms will want to know.

#ifdef __cplusplus
extern "C" {
#endif

struct DeviceStatus {
	// time shutter is open, in microseconds
	double shutter_speed;
	double analogue_gain;
	// 1.0/distance-in-metres, or 0 if unknown
	double lens_position;
	// 1/f so that brightness quadruples when this doubles, or 0 if unknown
	double aperture;
	// proportional to brightness with 0 = no flash, 1 = maximum flash
	double flash_intensity;
};

#ifdef __cplusplus
}
#endif
