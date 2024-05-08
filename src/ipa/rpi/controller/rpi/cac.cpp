/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * Chromatic Aberration Correction algorithm
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

static bool arrayToSet(const libcamera::YamlObject &params, std::vector<double> &inputArray, const Size &size)
{
	int num = 0;
	int max_num = (size.width + 1) * (size.height + 1);
	inputArray.resize(max_num);

	for (const auto &p : params.asList()) {
		if (num == max_num)
			return false;
		inputArray[num++] = p.get<double>(0);
	}

	return num == max_num;
}

static void setStrength(std::vector<double> &inputArray, std::vector<double> &outputArray,
			double strengthFactor)
{
	int num = 0;
	for (const auto &p : inputArray) {
		outputArray[num++] = p * strengthFactor;
	}
}

int Cac::read(const libcamera::YamlObject &params)
{
	config_.enabled = params.contains("lut_rx") && params.contains("lut_ry") &&
			  params.contains("lut_bx") && params.contains("lut_by");
	if (!config_.enabled)
		return 0;

	const Size &size = getHardwareConfig().cacRegions;

	if (!arrayToSet(params["lut_rx"], config_.lutRx, size)) {
		LOG(RPiCac, Error) << "Bad CAC lut_rx table";
		return -EINVAL;
	}

	if (!arrayToSet(params["lut_ry"], config_.lutRy, size)) {
		LOG(RPiCac, Error) << "Bad CAC lut_ry table";
		return -EINVAL;
	}

	if (!arrayToSet(params["lut_bx"], config_.lutBx, size)) {
		LOG(RPiCac, Error) << "Bad CAC lut_bx table";
		return -EINVAL;
	}

	if (!arrayToSet(params["lut_by"], config_.lutBy, size)) {
		LOG(RPiCac, Error) << "Bad CAC lut_by table";
		return -EINVAL;
	}

	double strength = params["strength"].get<double>(1);
	cacStatus_.lutRx = config_.lutRx;
	cacStatus_.lutRy = config_.lutRy;
	cacStatus_.lutBx = config_.lutBx;
	cacStatus_.lutBy = config_.lutBy;
	setStrength(config_.lutRx, cacStatus_.lutRx, strength);
	setStrength(config_.lutBx, cacStatus_.lutBx, strength);
	setStrength(config_.lutRy, cacStatus_.lutRy, strength);
	setStrength(config_.lutBy, cacStatus_.lutBy, strength);

	return 0;
}

void Cac::prepare(Metadata *imageMetadata)
{
	if (config_.enabled)
		imageMetadata->set("cac.status", cacStatus_);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Cac(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
