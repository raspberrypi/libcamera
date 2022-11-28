/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af_status.h - AF control algorithm status
 */
#pragma once

#include <libcamera/base/utils.h>

/*
 * The AF algorithm should post the following structure into the image's
 * "af.status" metadata. lensDriven and lensSetting should control the lens.
 */

struct AfStatus {
	enum AfState {
		STATE_UNKNOWN,  /* Initial state before trigger() or enableCAF() */
		STATE_SCANNING, /* Scan in progress or CAF large lens movement   */
		STATE_FOCUSED,  /* Scan succeeded or CAF has (nearly) converged  */
		STATE_FAILED    /* Scan failed or CAF lens position out of range */
	};

	AfState state;
	bool    lensKnown;    /* lensEstimate gives an estimate of focus in dioptres */
	bool    lensDriven;   /* lensSetting should be sent to the lens driver       */
	float   lensEstimate; /* Estimated current lens position in dioptres         */
	int32_t lensSetting;  /* Desired new lens position in HW units               */
};
