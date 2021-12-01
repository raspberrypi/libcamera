/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb_status.h - AWB control algorithm status
 */
#pragma once

// The AWB algorithm places its results into both the image and global metadata,
// under the tag "awb.status".

#ifdef __cplusplus
extern "C" {
#endif

struct AwbStatus {
	char mode[32];
	double temperature_K;
	double gain_r;
	double gain_g;
	double gain_b;
};

#ifdef __cplusplus
}
#endif
