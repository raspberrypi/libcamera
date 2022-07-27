/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ccm_status.h - CCM (colour correction matrix) control algorithm status
 */
#pragma once

/* The "ccm" algorithm generates an appropriate colour matrix. */

struct CcmStatus {
	double matrix[9];
	double saturation;
};
