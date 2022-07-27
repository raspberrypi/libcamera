/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * focus_status.h - focus measurement status
 */
#pragma once

#include <linux/bcm2835-isp.h>

/*
 * The focus algorithm should post the following structure into the image's
 * "focus.status" metadata. Recall that it's only reporting focus (contrast)
 * measurements, it's not driving any kind of auto-focus algorithm!
 */

struct FocusStatus {
	unsigned int num;
	uint32_t focusMeasures[FOCUS_REGIONS];
};
