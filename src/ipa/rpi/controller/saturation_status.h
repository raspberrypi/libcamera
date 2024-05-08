/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Saturation control algorithm status
 */
#pragma once

struct SaturationStatus {
	uint8_t shiftR;
	uint8_t shiftG;
	uint8_t shiftB;
};
