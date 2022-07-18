/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * sdn.cpp - SDN (spatial denoise) control algorithm
 */

#include <libcamera/base/log.h>

#include "../denoise_status.h"
#include "../noise_status.h"

#include "sdn.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSdn)

/*
 * Calculate settings for the spatial denoise block using the noise profile in
 * the image metadata.
 */

#define NAME "rpi.sdn"

Sdn::Sdn(Controller *controller)
	: DenoiseAlgorithm(controller), mode_(DenoiseMode::ColourOff)
{
}

char const *Sdn::name() const
{
	return NAME;
}

int Sdn::read(const libcamera::YamlObject &params)
{
	deviation_ = params["deviation"].get<double>(3.2);
	strength_ = params["strength"].get<double>(0.75);
	return 0;
}

void Sdn::initialise()
{
}

void Sdn::prepare(Metadata *imageMetadata)
{
	struct NoiseStatus noiseStatus = {};
	noiseStatus.noiseSlope = 3.0; /* in case no metadata */
	if (imageMetadata->get("noise.status", noiseStatus) != 0)
		LOG(RPiSdn, Warning) << "no noise profile found";
	LOG(RPiSdn, Debug)
		<< "Noise profile: constant " << noiseStatus.noiseConstant
		<< " slope " << noiseStatus.noiseSlope;
	struct DenoiseStatus status;
	status.noiseConstant = noiseStatus.noiseConstant * deviation_;
	status.noiseSlope = noiseStatus.noiseSlope * deviation_;
	status.strength = strength_;
	status.mode = static_cast<std::underlying_type_t<DenoiseMode>>(mode_);
	imageMetadata->set("denoise.status", status);
	LOG(RPiSdn, Debug)
		<< "programmed constant " << status.noiseConstant
		<< " slope " << status.noiseSlope
		<< " strength " << status.strength;
}

void Sdn::setMode(DenoiseMode mode)
{
	/* We only distinguish between off and all other modes. */
	mode_ = mode;
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Sdn(controller);
}
static RegisterAlgorithm reg(NAME, &create);
