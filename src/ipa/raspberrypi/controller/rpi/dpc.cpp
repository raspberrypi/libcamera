/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * dpc.cpp - DPC (defective pixel correction) control algorithm
 */

#include "../logging.hpp"
#include "dpc.hpp"

using namespace RPi;

// We use the lux status so that we can apply stronger settings in darkness (if
// necessary).

#define NAME "rpi.dpc"

Dpc::Dpc(Controller *controller)
	: Algorithm(controller)
{
}

char const *Dpc::Name() const
{
	return NAME;
}

void Dpc::Read(boost::property_tree::ptree const &params)
{
	config_.strength = params.get<int>("strength", 1);
	if (config_.strength < 0 || config_.strength > 2)
		throw std::runtime_error("Dpc: bad strength value");
}

void Dpc::Prepare(Metadata *image_metadata)
{
	DpcStatus dpc_status = {};
	// Should we vary this with lux level or analogue gain? TBD.
	dpc_status.strength = config_.strength;
	RPI_LOG("Dpc: strength " << dpc_status.strength);
	image_metadata->Set("dpc.status", dpc_status);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Dpc(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
