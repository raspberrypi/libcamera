/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * agc_status.h - AGC/AEC control algorithm status
 */
#pragma once

#include <libcamera/base/utils.h>

/*
 * The AGC algorithm should post the following structure into the image's
 * "agc.status" metadata.
 */

/*
 * Note: total_exposure_value will be reported as zero until the algorithm has
 * seen statistics and calculated meaningful values. The contents should be
 * ignored until then.
 */

struct AgcStatus {
	libcamera::utils::Duration totalExposureValue; /* value for all exposure and gain for this image */
	libcamera::utils::Duration targetExposureValue; /* (unfiltered) target total exposure AGC is aiming for */
	libcamera::utils::Duration shutterTime;
	double analogueGain;
	char exposureMode[32];
	char constraintMode[32];
	char meteringMode[32];
	double ev;
	libcamera::utils::Duration flickerPeriod;
	int floatingRegionEnable;
	libcamera::utils::Duration fixedShutter;
	double fixedAnalogueGain;
	double digitalGain;
	int locked;
};
