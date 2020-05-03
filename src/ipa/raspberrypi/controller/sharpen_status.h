/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sharpen_status.h - Sharpen control algorithm status
 */
#pragma once

// The "sharpen" algorithm stores the strength to use.

#ifdef __cplusplus
extern "C" {
#endif

struct SharpenStatus {
	// controls the smallest level of detail (or noise!) that sharpening will pick up
	double threshold;
	// the rate at which the sharpening response ramps once above the threshold
	double strength;
	// upper limit of the allowed sharpening response
	double limit;
};

#ifdef __cplusplus
}
#endif
