/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Limited
 *
 * denoise_status.h - Denoise control algorithm status
 */
#pragma once

// This stores the parameters required for Denoise.

#ifdef __cplusplus
extern "C" {
#endif

struct DenoiseStatus {
	double noise_constant;
	double noise_slope;
	double strength;
	unsigned int mode;
};

#ifdef __cplusplus
}
#endif
