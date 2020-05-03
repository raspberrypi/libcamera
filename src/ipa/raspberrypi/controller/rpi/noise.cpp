/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * noise.cpp - Noise control algorithm
 */

#include <math.h>

#include "../device_status.h"
#include "../logging.hpp"
#include "../noise_status.h"

#include "noise.hpp"

using namespace RPi;

#define NAME "rpi.noise"

Noise::Noise(Controller *controller)
	: Algorithm(controller), mode_factor_(1.0)
{
}

char const *Noise::Name() const
{
	return NAME;
}

void Noise::SwitchMode(CameraMode const &camera_mode)
{
	// For example, we would expect a 2x2 binned mode to have a "noise
	// factor" of sqrt(2x2) = 2. (can't be less than one, right?)
	mode_factor_ = std::max(1.0, camera_mode.noise_factor);
}

void Noise::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG(Name());
	reference_constant_ = params.get<double>("reference_constant");
	reference_slope_ = params.get<double>("reference_slope");
}

void Noise::Prepare(Metadata *image_metadata)
{
	struct DeviceStatus device_status;
	device_status.analogue_gain = 1.0; // keep compiler calm
	if (image_metadata->Get("device.status", device_status) == 0) {
		// There is a slight question as to exactly how the noise
		// profile, specifically the constant part of it, scales. For
		// now we assume it all scales the same, and we'll revisit this
		// if it proves substantially wrong.  NOTE: we may also want to
		// make some adjustments based on the camera mode (such as
		// binning), if we knew how to discover it...
		double factor = sqrt(device_status.analogue_gain) / mode_factor_;
		struct NoiseStatus status;
		status.noise_constant = reference_constant_ * factor;
		status.noise_slope = reference_slope_ * factor;
		image_metadata->Set("noise.status", status);
		RPI_LOG(Name() << ": constant " << status.noise_constant
			       << " slope " << status.noise_slope);
	} else
		RPI_WARN(Name() << " no metadata");
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return new Noise(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
