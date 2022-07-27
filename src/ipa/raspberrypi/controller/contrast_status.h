/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * contrast_status.h - contrast (gamma) control algorithm status
 */
#pragma once

/*
 * The "contrast" algorithm creates a gamma curve, optionally doing a little bit
 * of contrast stretching based on the AGC histogram.
 */

constexpr unsigned int ContrastNumPoints = 33;

struct ContrastPoint {
	uint16_t x;
	uint16_t y;
};

struct ContrastStatus {
	struct ContrastPoint points[ContrastNumPoints];
	double brightness;
	double contrast;
};
