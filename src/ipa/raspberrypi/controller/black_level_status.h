/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * black_level_status.h - black level control algorithm status
 */
#pragma once

/* The "black level" algorithm stores the black levels to use. */

struct BlackLevelStatus {
	uint16_t blackLevelR; /* out of 16 bits */
	uint16_t blackLevelG;
	uint16_t blackLevelB;
};
