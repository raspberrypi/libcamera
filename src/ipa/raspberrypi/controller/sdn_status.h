/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sdn_status.h - SDN (spatial denoise) control algorithm status
 */
#pragma once

// This stores the parameters required for Spatial Denoise (SDN).

#ifdef __cplusplus
extern "C" {
#endif

struct SdnStatus {
	double noise_constant;
	double noise_slope;
	double strength;
};

#ifdef __cplusplus
}
#endif
