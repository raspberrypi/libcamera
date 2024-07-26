/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Tonemap control algorithm
 */
#include "tonemap.h"

#include <libcamera/base/log.h>

#include "tonemap_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiTonemap)

#define NAME "rpi.tonemap"

Tonemap::Tonemap(Controller *controller)
	: Algorithm(controller)
{
}

char const *Tonemap::name() const
{
	return NAME;
}

int Tonemap::read(const libcamera::YamlObject &params)
{
	config_.detailConstant = params["detail_constant"].get<uint16_t>(0);
	config_.detailSlope = params["detail_slope"].get<double>(0.1);
	config_.iirStrength = params["iir_strength"].get<double>(1.0);
	config_.strength = params["strength"].get<double>(1.0);
	config_.tonemap = params["tone_curve"].get<ipa::Pwl>(ipa::Pwl{});
	return 0;
}

void Tonemap::initialise()
{
}

void Tonemap::prepare(Metadata *imageMetadata)
{
	TonemapStatus tonemapStatus;

	tonemapStatus.detailConstant = config_.detailConstant;
	tonemapStatus.detailSlope = config_.detailSlope;
	tonemapStatus.iirStrength = config_.iirStrength;
	tonemapStatus.strength = config_.strength;
	tonemapStatus.tonemap = config_.tonemap;
	imageMetadata->set("tonemap.status", tonemapStatus);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Tonemap(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
