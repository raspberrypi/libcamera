/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * noise_status.h - Noise control algorithm status
 */
#pragma once

// The "noise" algorithm stores an estimate of the noise profile for this image.

#ifdef __cplusplus
extern "C" {
#endif

struct NoiseStatus {
	double noise_constant;
	double noise_slope;
};

#ifdef __cplusplus
}
#endif
