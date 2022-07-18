/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * noise.cpp - Noise control algorithm
 */

#include <math.h>

#include <libcamera/base/log.h>

#include "../device_status.h"
#include "../noise_status.h"

#include "noise.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiNoise)

#define NAME "rpi.noise"

Noise::Noise(Controller *controller)
	: Algorithm(controller), modeFactor_(1.0)
{
}

char const *Noise::name() const
{
	return NAME;
}

void Noise::switchMode(CameraMode const &cameraMode,
		       [[maybe_unused]] Metadata *metadata)
{
	/*
	 * For example, we would expect a 2x2 binned mode to have a "noise
	 * factor" of sqrt(2x2) = 2. (can't be less than one, right?)
	 */
	modeFactor_ = std::max(1.0, cameraMode.noiseFactor);
}

int Noise::read(const libcamera::YamlObject &params)
{
	auto value = params["reference_constant"].get<double>();
	if (!value)
		return -EINVAL;
	referenceConstant_ = *value;

	value = params["reference_slope"].get<double>();
	if (!value)
		return -EINVAL;
	referenceSlope_ = *value;

	return 0;
}

void Noise::prepare(Metadata *imageMetadata)
{
	struct DeviceStatus deviceStatus;
	deviceStatus.analogueGain = 1.0; /* keep compiler calm */
	if (imageMetadata->get("device.status", deviceStatus) == 0) {
		/*
		 * There is a slight question as to exactly how the noise
		 * profile, specifically the constant part of it, scales. For
		 * now we assume it all scales the same, and we'll revisit this
		 * if it proves substantially wrong.  NOTE: we may also want to
		 * make some adjustments based on the camera mode (such as
		 * binning), if we knew how to discover it...
		 */
		double factor = sqrt(deviceStatus.analogueGain) / modeFactor_;
		struct NoiseStatus status;
		status.noiseConstant = referenceConstant_ * factor;
		status.noiseSlope = referenceSlope_ * factor;
		imageMetadata->set("noise.status", status);
		LOG(RPiNoise, Debug)
			<< "constant " << status.noiseConstant
			<< " slope " << status.noiseSlope;
	} else
		LOG(RPiNoise, Warning) << " no metadata";
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new Noise(controller);
}
static RegisterAlgorithm reg(NAME, &create);
