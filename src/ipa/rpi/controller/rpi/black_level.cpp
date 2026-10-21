/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * black level control algorithm
 */

#include <stdint.h>

#include <libcamera/base/log.h>

#include "../black_level_status.h"

#include "black_level.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiBlackLevel)

#define NAME "rpi.black_level"

BlackLevel::BlackLevel(Controller *controller)
	: BlackLevelAlgorithm(controller)
{
}

char const *BlackLevel::name() const
{
	return NAME;
}

int BlackLevel::read(const libcamera::ValueNode &params)
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

	/* Allow "black_level_func" as a shorthand for all 3 colours. */
	libcamera::ipa::Pwl blackLevelFunc;
	blackLevelFunc = params["black_level_func"].get<ipa::Pwl>(ipa::Pwl{});
	blackLevelFuncR_ = params["black_level_func_r"].get<ipa::Pwl>(blackLevelFunc);
	blackLevelFuncG_ = params["black_level_func_g"].get<ipa::Pwl>(blackLevelFunc);
	blackLevelFuncB_ = params["black_level_func_b"].get<ipa::Pwl>(blackLevelFunc);

	/* Warn the user if they've defined constant black levels but which are unused. */
	if (!blackLevelFuncR_.empty() && !blackLevelFuncG_.empty() &&
	    !blackLevelFuncB_.empty() && params.contains("black_level"))
		LOG(RPiBlackLevel, Warning) << "\"black_level\" specified but not used";

	if (!blackLevelFuncR_.empty() && params.contains("black_level_r"))
		LOG(RPiBlackLevel, Warning) << "\"black_level_r\" specified but not used";
	if (!blackLevelFuncG_.empty() && params.contains("black_level_g"))
		LOG(RPiBlackLevel, Warning) << "\"black_level_g\" specified but not used";
	if (!blackLevelFuncB_.empty() && params.contains("black_level_b"))
		LOG(RPiBlackLevel, Warning) << "\"black_level_b\" specified but not used";

	return 0;
}

void BlackLevel::initialValues(uint16_t &blackLevelR, uint16_t &blackLevelG,
			       uint16_t &blackLevelB)
{
	if (!blackLevelFuncR_.empty())
		blackLevelR_ = blackLevelFuncR_.eval(blackLevelFuncR_.domain().clamp(1.0));

	if (!blackLevelFuncG_.empty())
		blackLevelG_ = blackLevelFuncG_.eval(blackLevelFuncG_.domain().clamp(1.0));

	if (!blackLevelFuncB_.empty())
		blackLevelB_ = blackLevelFuncB_.eval(blackLevelFuncB_.domain().clamp(1.0));

	blackLevelR = blackLevelR_;
	blackLevelG = blackLevelG_;
	blackLevelB = blackLevelB_;
}

void BlackLevel::prepare(Metadata *imageMetadata)
{
	DeviceStatus deviceStatus;
	if (!imageMetadata->get("device.status", deviceStatus)) {
		if (!blackLevelFuncR_.empty())
			blackLevelR_ = blackLevelFuncR_.eval(blackLevelFuncR_.domain().clamp(deviceStatus.analogueGain));

		if (!blackLevelFuncG_.empty())
			blackLevelG_ = blackLevelFuncG_.eval(blackLevelFuncG_.domain().clamp(deviceStatus.analogueGain));

		if (!blackLevelFuncB_.empty())
			blackLevelB_ = blackLevelFuncB_.eval(blackLevelFuncB_.domain().clamp(deviceStatus.analogueGain));
	}

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
