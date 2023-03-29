/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * hdr.h - Tonemap control algorithm status
 */
#pragma once

#include "pwl.h"

struct TonemapStatus {
	uint16_t detailConstant;
	double detailSlope;
	double iirStrength;
	double strength;
	RPiController::Pwl tonemap;
};
