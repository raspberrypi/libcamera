/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Limited
 *
 * device_status.h - device (image sensor) status
 */
#pragma once

#include <iostream>

#include <libcamera/base/utils.h>

/*
 * Definition of "device metadata" which stores things like shutter time and
 * analogue gain that downstream control algorithms will want to know.
 */

struct DeviceStatus {
	DeviceStatus()
		: shutter_speed(std::chrono::seconds(0)), frame_length(0),
		  analogue_gain(0.0), lens_position(0.0), aperture(0.0),
		  flash_intensity(0.0)
	{
	}

	friend std::ostream &operator<<(std::ostream &out, const DeviceStatus &d);

	/* time shutter is open */
	libcamera::utils::Duration shutter_speed;
	/* frame length given in number of lines */
	uint32_t frame_length;
	double analogue_gain;
	/* 1.0/distance-in-metres, or 0 if unknown */
	double lens_position;
	/* 1/f so that brightness quadruples when this doubles, or 0 if unknown */
	double aperture;
	/* proportional to brightness with 0 = no flash, 1 = maximum flash */
	double flash_intensity;
};
