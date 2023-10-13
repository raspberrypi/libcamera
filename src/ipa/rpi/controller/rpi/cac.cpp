/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * cac.cpp - Chromatic Aberration Correction algorithm
 */
#include "cac.h"

#include <libcamera/base/log.h>

#include "cac_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiCac)

#define NAME "rpi.cac"

Cac::Cac(Controller *controller)
	: Algorithm(controller)
{
}

char const *Cac::name() const
{
	return NAME;
}

int Cac::read(const libcamera::YamlObject &params)
{
	arrayToSet(params["lut_rx"], config_.lutRx);
	arrayToSet(params["lut_ry"], config_.lutRy);
	arrayToSet(params["lut_bx"], config_.lutBx);
	arrayToSet(params["lut_by"], config_.lutBy);
	cacStatus_.lutRx = config_.lutRx;
	cacStatus_.lutRy = config_.lutRy;
	cacStatus_.lutBx = config_.lutBx;
	cacStatus_.lutBy = config_.lutBy;
	double strength = params["strength"].get<double>(1);
	setStrength(config_.lutRx, cacStatus_.lutRx, strength);
	setStrength(config_.lutBx, cacStatus_.lutBx, strength);
	setStrength(config_.lutRy, cacStatus_.lutRy, strength);
	setStrength(config_.lutBy, cacStatus_.lutBy, strength);
	return 0;
}

void Cac::initialise()
{
}

void Cac::arrayToSet(const libcamera::YamlObject &params, std::vector<double> &inputArray)
{
	int num = 0;
	const Size &size = getHardwareConfig().cacRegions;
	inputArray.resize((size.width + 1) * (size.height + 1));
	for (const auto &p : params.asList()) {
		inputArray[num++] = p.get<double>(0);
	}
}

void Cac::setStrength(std::vector<double> &inputArray, std::vector<double> &outputArray,
		      double strengthFactor)
{
	int num = 0;
	for (const auto &p : inputArray) {
		outputArray[num++] = p * strengthFactor;
	}
}

void Cac::prepare(Metadata *imageMetadata)
{
	imageMetadata->set("cac.status", cacStatus_);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Cac(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
