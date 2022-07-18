/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * black_level.cpp - black level control algorithm
 */

#include <math.h>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "../black_level_status.h"

#include "black_level.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiBlackLevel)

#define NAME "rpi.black_level"

BlackLevel::BlackLevel(Controller *controller)
	: Algorithm(controller)
{
}

char const *BlackLevel::name() const
{
	return NAME;
}

int BlackLevel::read(const libcamera::YamlObject &params)
{
	/* 64 in 10 bits scaled to 16 bits */
	uint16_t blackLevel = params["black_level"].get<uint16_t>(4096);
	blackLevelR_ = params["black_level_r"].get<uint16_t>(blackLevel);
	blackLevelG_ = params["black_level_g"].get<uint16_t>(blackLevel);
	blackLevelB_ = params["black_level_b"].get<uint16_t>(blackLevel);
	LOG(RPiBlackLevel, Debug)
		<< " Read black levels red " << blackLevelR_
		<< " green " << blackLevelG_
		<< " blue " << blackLevelB_;
	return 0;
}

void BlackLevel::prepare(Metadata *imageMetadata)
{
	/*
	 * Possibly we should think about doing this in a switchMode or
	 * something?
	 */
	struct BlackLevelStatus status;
	status.blackLevelR = blackLevelR_;
	status.blackLevelG = blackLevelG_;
	status.blackLevelB = blackLevelB_;
	imageMetadata->set("black_level.status", status);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new BlackLevel(controller);
}
static RegisterAlgorithm reg(NAME, &create);
