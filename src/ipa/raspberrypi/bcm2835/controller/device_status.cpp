/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * device_status.cpp - device (image sensor) status
 */
#include "device_status.h"

using namespace libcamera; /* for the Duration operator<< overload */

std::ostream &operator<<(std::ostream &out, const DeviceStatus &d)
{
	out << "Exposure: " << d.shutter_speed
	    << " Frame length: " << d.frame_length
	    << " Gain: " << d.analogue_gain
	    << " Aperture: " << d.aperture
	    << " Lens: " << d.lens_position
	    << " Flash: " << d.flash_intensity;

	return out;
}
