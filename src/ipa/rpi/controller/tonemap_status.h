/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Tonemap control algorithm status
 */
#pragma once

#include <libipa/pwl.h>

struct TonemapStatus {
	uint16_t detailConstant;
	double detailSlope;
	double iirStrength;
	double strength;
	libcamera::ipa::Pwl tonemap;
};
