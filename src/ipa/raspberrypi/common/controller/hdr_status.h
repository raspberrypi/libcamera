/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * hdr_status.h - HDR control algorithm status
 */
#pragma once

struct HdrStatus {
	uint16_t thresholdLo;
	uint8_t diffPower;
	double motionThreshold;
};
