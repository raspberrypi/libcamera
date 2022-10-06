/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * device_status.h - device (image sensor) status
 */
#pragma once

#include <iostream>
#include <optional>

#include <libcamera/base/utils.h>

/*
 * Definition of "device metadata" which stores things like shutter time and
 * analogue gain that downstream control algorithms will want to know.
 */

struct DeviceStatus {
	DeviceStatus()
		: shutterSpeed(std::chrono::seconds(0)), frameLength(0),
		  lineLength(std::chrono::seconds(0)), analogueGain(0.0)
	{
	}

	friend std::ostream &operator<<(std::ostream &out, const DeviceStatus &d);

	/* time shutter is open */
	libcamera::utils::Duration shutterSpeed;
	/* frame length given in number of lines */
	uint32_t frameLength;
	/* line length for the current frame */
	libcamera::utils::Duration lineLength;
	double analogueGain;
	/* 1.0/distance-in-metres, or 0 if unknown */
	std::optional<double> lensPosition;
	/* 1/f so that brightness quadruples when this doubles, or 0 if unknown */
	std::optional<double> aperture;
	/* proportional to brightness with 0 = no flash, 1 = maximum flash */
	std::optional<double> flashIntensity;
	/* Sensor reported temperature value (in degrees) */
	std::optional<double> sensorTemperature;
};
