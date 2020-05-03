/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * contrast_status.h - contrast (gamma) control algorithm status
 */
#pragma once

// The "contrast" algorithm creates a gamma curve, optionally doing a little bit
// of contrast stretching based on the AGC histogram.

#ifdef __cplusplus
extern "C" {
#endif

#define CONTRAST_NUM_POINTS 33

struct ContrastPoint {
	uint16_t x;
	uint16_t y;
};

struct ContrastStatus {
	struct ContrastPoint points[CONTRAST_NUM_POINTS];
	double brightness;
	double contrast;
};

#ifdef __cplusplus
}
#endif
