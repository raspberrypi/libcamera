/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * stitch control algorithm status
 */
#pragma once

/*
 * Parameters for the stitch block.
 */

struct StitchStatus {
	uint16_t thresholdLo;
	uint8_t diffPower;
	double motionThreshold;
};
