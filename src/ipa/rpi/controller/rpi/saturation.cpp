/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Saturation control algorithm
 */
#include "saturation.h"

#include <libcamera/base/log.h>

#include "saturation_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSaturation)

#define NAME "rpi.saturation"

Saturation::Saturation(Controller *controller)
	: Algorithm(controller)
{
}

char const *Saturation::name() const
{
	return NAME;
}

int Saturation::read(const libcamera::YamlObject &params)
{
	config_.shiftR = params["shift_r"].get<uint8_t>(0);
	config_.shiftG = params["shift_g"].get<uint8_t>(0);
	config_.shiftB = params["shift_b"].get<uint8_t>(0);
	return 0;
}

void Saturation::initialise()
{
}

void Saturation::prepare(Metadata *imageMetadata)
{
	SaturationStatus saturation;

	saturation.shiftR = config_.shiftR;
	saturation.shiftG = config_.shiftG;
	saturation.shiftB = config_.shiftB;
	imageMetadata->set("saturation.status", saturation);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Saturation(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
