/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * contrast_status.h - contrast (gamma) control algorithm status
 */
#pragma once

#include "pwl.h"

/*
 * The "contrast" algorithm creates a gamma curve, optionally doing a little bit
 * of contrast stretching based on the AGC histogram.
 */

struct ContrastStatus {
	RPiController::Pwl gammaCurve;
	double brightness;
	double contrast;
};
