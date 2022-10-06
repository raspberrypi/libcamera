/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * device_status.cpp - device (image sensor) status
 */
#include "device_status.h"

using namespace libcamera; /* for the Duration operator<< overload */

std::ostream &operator<<(std::ostream &out, const DeviceStatus &d)
{
	out << "Exposure: " << d.shutterSpeed
	    << " Frame length: " << d.frameLength
	    << " Line length: " << d.lineLength
	    << " Gain: " << d.analogueGain;

	if (d.aperture)
		out << " Aperture: " << *d.aperture;

	if (d.lensPosition)
		out << " Lens: " << *d.lensPosition;

	if (d.flashIntensity)
		out << " Flash: " << *d.flashIntensity;

	if (d.sensorTemperature)
		out << " Temperature: " << *d.sensorTemperature;

	return out;
}
