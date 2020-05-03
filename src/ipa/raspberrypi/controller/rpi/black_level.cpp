/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * black_level.cpp - black level control algorithm
 */

#include <math.h>
#include <stdint.h>

#include "../black_level_status.h"
#include "../logging.hpp"

#include "black_level.hpp"

using namespace RPi;

#define NAME "rpi.black_level"

BlackLevel::BlackLevel(Controller *controller)
	: Algorithm(controller)
{
}

char const *BlackLevel::Name() const
{
	return NAME;
}

void BlackLevel::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG(Name());
	uint16_t black_level = params.get<uint16_t>(
		"black_level", 4096); // 64 in 10 bits scaled to 16 bits
	black_level_r_ = params.get<uint16_t>("black_level_r", black_level);
	black_level_g_ = params.get<uint16_t>("black_level_g", black_level);
	black_level_b_ = params.get<uint16_t>("black_level_b", black_level);
}

void BlackLevel::Prepare(Metadata *image_metadata)
{
	// Possibly we should think about doing this in a switch_mode or
	// something?
	struct BlackLevelStatus status;
	status.black_level_r = black_level_r_;
	status.black_level_g = black_level_g_;
	status.black_level_b = black_level_b_;
	image_metadata->Set("black_level.status", status);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return new BlackLevel(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
