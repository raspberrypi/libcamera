/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * black_level_status.h - black level control algorithm status
 */
#pragma once

// The "black level" algorithm stores the black levels to use.

#ifdef __cplusplus
extern "C" {
#endif

struct BlackLevelStatus {
	uint16_t black_level_r; // out of 16 bits
	uint16_t black_level_g;
	uint16_t black_level_b;
};

#ifdef __cplusplus
}
#endif
