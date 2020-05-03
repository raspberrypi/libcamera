/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * dpc_status.h - DPC (defective pixel correction) control algorithm status
 */
#pragma once

// The "DPC" algorithm sets defective pixel correction strength.

#ifdef __cplusplus
extern "C" {
#endif

struct DpcStatus {
	int strength; // 0 = "off", 1 = "normal", 2 = "strong"
};

#ifdef __cplusplus
}
#endif
