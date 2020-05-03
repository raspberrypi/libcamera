/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sdn.cpp - SDN (spatial denoise) control algorithm
 */

#include "../noise_status.h"
#include "../sdn_status.h"

#include "sdn.hpp"

using namespace RPi;

// Calculate settings for the spatial denoise block using the noise profile in
// the image metadata.

#define NAME "rpi.sdn"

Sdn::Sdn(Controller *controller)
	: Algorithm(controller)
{
}

char const *Sdn::Name() const
{
	return NAME;
}

void Sdn::Read(boost::property_tree::ptree const &params)
{
	deviation_ = params.get<double>("deviation", 3.2);
	strength_ = params.get<double>("strength", 0.75);
}

void Sdn::Initialise() {}

void Sdn::Prepare(Metadata *image_metadata)
{
	struct NoiseStatus noise_status = {};
	noise_status.noise_slope = 3.0; // in case no metadata
	if (image_metadata->Get("noise.status", noise_status) != 0)
		RPI_WARN("Sdn: no noise profile found");
	RPI_LOG("Noise profile: constant " << noise_status.noise_constant
					   << " slope "
					   << noise_status.noise_slope);
	struct SdnStatus status;
	status.noise_constant = noise_status.noise_constant * deviation_;
	status.noise_slope = noise_status.noise_slope * deviation_;
	status.strength = strength_;
	image_metadata->Set("sdn.status", status);
	RPI_LOG("Sdn: programmed constant " << status.noise_constant
					    << " slope " << status.noise_slope
					    << " strength "
					    << status.strength);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Sdn(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
