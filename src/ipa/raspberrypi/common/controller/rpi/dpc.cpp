/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * dpc.cpp - DPC (defective pixel correction) control algorithm
 */

#include <libcamera/base/log.h>

#include "dpc.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiDpc)

/*
 * We use the lux status so that we can apply stronger settings in darkness (if
 * necessary).
 */

#define NAME "rpi.dpc"

Dpc::Dpc(Controller *controller)
	: Algorithm(controller)
{
}

char const *Dpc::name() const
{
	return NAME;
}

int Dpc::read(const libcamera::YamlObject &params)
{
	config_.strength = params["strength"].get<int>(1);
	if (config_.strength < 0 || config_.strength > 2) {
		LOG(RPiDpc, Error) << "Bad strength value";
		return -EINVAL;
	}

	return 0;
}

void Dpc::prepare(Metadata *imageMetadata)
{
	DpcStatus dpcStatus = {};
	/* Should we vary this with lux level or analogue gain? TBD. */
	dpcStatus.strength = config_.strength;
	LOG(RPiDpc, Debug) << "strength " << dpcStatus.strength;
	imageMetadata->set("dpc.status", dpcStatus);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Dpc(controller);
}
static RegisterAlgorithm reg(NAME, &create);
