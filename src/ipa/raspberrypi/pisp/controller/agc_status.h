/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc_status.h - AGC/AEC control algorithm status
 */
#pragma once

#include <libcamera/base/utils.h>

// The AGC algorithm should post the following structure into the image's
// "agc.status" metadata.

#ifdef __cplusplus
extern "C" {
#endif

// Note: total_exposure_value will be reported as zero until the algorithm has
// seen statistics and calculated meaningful values. The contents should be
// ignored until then.

struct AgcStatus {
	libcamera::utils::Duration total_exposure_value; // value for all exposure and gain for this image
	libcamera::utils::Duration target_exposure_value; // (unfiltered) target total exposure AGC is aiming for
	libcamera::utils::Duration shutter_time;
	double analogue_gain;
	char exposure_mode[32];
	char constraint_mode[32];
	char metering_mode[32];
	double ev;
	libcamera::utils::Duration flicker_period;
	int floating_region_enable;
	libcamera::utils::Duration fixed_shutter;
	double fixed_analogue_gain;
	double digital_gain;
	int locked;
};

#ifdef __cplusplus
}
#endif
