/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * Denoise control algorithm status
 */
#pragma once

/* This stores the parameters required for Denoise. */

struct DenoiseStatus {
	double noiseConstant;
	double noiseSlope;
	double strength;
	unsigned int mode;
};

struct SdnStatus {
	double noiseConstant;
	double noiseSlope;
	double noiseConstant2;
	double noiseSlope2;
	double strength;
};

struct CdnStatus {
	double strength;
	double threshold;
};

struct TdnStatus {
	double noiseConstant;
	double noiseSlope;
	double threshold;
};
