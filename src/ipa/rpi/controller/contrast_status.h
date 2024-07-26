/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * contrast (gamma) control algorithm status
 */
#pragma once

#include "libipa/pwl.h"

/*
 * The "contrast" algorithm creates a gamma curve, optionally doing a little bit
 * of contrast stretching based on the AGC histogram.
 */

struct ContrastStatus {
	libcamera::ipa::Pwl gammaCurve;
	double brightness;
	double contrast;
};
