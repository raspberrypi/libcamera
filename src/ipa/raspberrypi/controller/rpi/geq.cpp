/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * geq.cpp - GEQ (green equalisation) control algorithm
 */

#include <libcamera/base/log.h>

#include "../device_status.h"
#include "../lux_status.h"
#include "../pwl.h"

#include "geq.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiGeq)

/*
 * We use the lux status so that we can apply stronger settings in darkness (if
 * necessary).
 */

#define NAME "rpi.geq"

Geq::Geq(Controller *controller)
	: Algorithm(controller)
{
}

char const *Geq::name() const
{
	return NAME;
}

int Geq::read(const libcamera::YamlObject &params)
{
	config_.offset = params["offset"].get<uint16_t>(0);
	config_.slope = params["slope"].get<double>(0.0);
	if (config_.slope < 0.0 || config_.slope >= 1.0) {
		LOG(RPiGeq, Error) << "Bad slope value";
		return -EINVAL;
	}

	if (params.contains("strength")) {
		int ret = config_.strength.read(params["strength"]);
		if (ret)
			return ret;
	}

	return 0;
}

void Geq::prepare(Metadata *imageMetadata)
{
	LuxStatus luxStatus = {};
	luxStatus.lux = 400;
	if (imageMetadata->get("lux.status", luxStatus))
		LOG(RPiGeq, Warning) << "no lux data found";
	DeviceStatus deviceStatus;
	deviceStatus.analogueGain = 1.0; /* in case not found */
	if (imageMetadata->get("device.status", deviceStatus))
		LOG(RPiGeq, Warning)
			<< "no device metadata - use analogue gain of 1x";
	GeqStatus geqStatus = {};
	double strength = config_.strength.empty()
			? 1.0
			: config_.strength.eval(config_.strength.domain().clip(luxStatus.lux));
	strength *= deviceStatus.analogueGain;
	double offset = config_.offset * strength;
	double slope = config_.slope * strength;
	geqStatus.offset = std::min(65535.0, std::max(0.0, offset));
	geqStatus.slope = std::min(.99999, std::max(0.0, slope));
	LOG(RPiGeq, Debug)
		<< "offset " << geqStatus.offset << " slope "
		<< geqStatus.slope << " (analogue gain "
		<< deviceStatus.analogueGain << " lux "
		<< luxStatus.lux << ")";
	imageMetadata->set("geq.status", geqStatus);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Geq(controller);
}
static RegisterAlgorithm reg(NAME, &create);
