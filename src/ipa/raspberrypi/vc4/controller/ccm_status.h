/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ccm_status.h - CCM (colour correction matrix) control algorithm status
 */
#pragma once

// The "ccm" algorithm generates an appropriate colour matrix.

#ifdef __cplusplus
extern "C" {
#endif

struct CcmStatus {
	double matrix[9];
	double saturation;
};

#ifdef __cplusplus
}
#endif
