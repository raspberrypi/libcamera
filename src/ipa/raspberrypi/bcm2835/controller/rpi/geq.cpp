/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * geq.cpp - GEQ (green equalisation) control algorithm
 */

#include <libcamera/base/log.h>

#include "../device_status.h"
#include "../lux_status.h"
#include "../pwl.hpp"

#include "geq.hpp"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiGeq)

// We use the lux status so that we can apply stronger settings in darkness (if
// necessary).

#define NAME "rpi.geq"

Geq::Geq(Controller *controller)
	: Algorithm(controller)
{
}

char const *Geq::Name() const
{
	return NAME;
}

void Geq::Read(boost::property_tree::ptree const &params)
{
	config_.offset = params.get<uint16_t>("offset", 0);
	config_.slope = params.get<double>("slope", 0.0);
	if (config_.slope < 0.0 || config_.slope >= 1.0)
		throw std::runtime_error("Geq: bad slope value");
	if (params.get_child_optional("strength"))
		config_.strength.Read(params.get_child("strength"));
}

void Geq::Prepare(Metadata *image_metadata)
{
	LuxStatus lux_status = {};
	lux_status.lux = 400;
	if (image_metadata->Get("lux.status", lux_status))
		LOG(RPiGeq, Warning) << "no lux data found";
	DeviceStatus device_status;
	device_status.analogue_gain = 1.0; // in case not found
	if (image_metadata->Get("device.status", device_status))
		LOG(RPiGeq, Warning)
			<< "no device metadata - use analogue gain of 1x";
	GeqStatus geq_status = {};
	double strength =
		config_.strength.Empty()
			? 1.0
			: config_.strength.Eval(config_.strength.Domain().Clip(
				  lux_status.lux));
	strength *= device_status.analogue_gain;
	double offset = config_.offset * strength;
	double slope = config_.slope * strength;
	geq_status.offset = std::min(65535.0, std::max(0.0, offset));
	geq_status.slope = std::min(.99999, std::max(0.0, slope));
	LOG(RPiGeq, Debug)
		<< "offset " << geq_status.offset << " slope "
		<< geq_status.slope << " (analogue gain "
		<< device_status.analogue_gain << " lux "
		<< lux_status.lux << ")";
	image_metadata->Set("geq.status", geq_status);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Geq(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
