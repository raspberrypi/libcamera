/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * AGC/AEC control algorithm status
 */
#pragma once

#include <string>

#include <libcamera/base/utils.h>

#include "hdr_status.h"

/*
 * The AGC algorithm process method should post an AgcStatus into the image
 * metadata under the tag "agc.status".
 * The AGC algorithm prepare method should post an AgcPrepareStatus instead
 * under "agc.prepare_status".
 */

/*
 * Note: total_exposure_value will be reported as zero until the algorithm has
 * seen statistics and calculated meaningful values. The contents should be
 * ignored until then.
 */

struct AgcStatus {
	libcamera::utils::Duration totalExposureValue; /* value for all exposure and gain for this image */
	libcamera::utils::Duration targetExposureValue; /* (unfiltered) target total exposure AGC is aiming for */
	libcamera::utils::Duration exposureTime;
	double analogueGain;
	std::string exposureMode;
	std::string constraintMode;
	std::string meteringMode;
	double ev;
	libcamera::utils::Duration flickerPeriod;
	int floatingRegionEnable;
	libcamera::utils::Duration fixedExposureTime;
	double fixedAnalogueGain;
	unsigned int channel;
	HdrStatus hdr;
};

struct AgcPrepareStatus {
	double digitalGain;
	int locked;
};
