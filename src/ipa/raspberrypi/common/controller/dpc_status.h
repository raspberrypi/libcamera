/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * dpc_status.h - DPC (defective pixel correction) control algorithm status
 */
#pragma once

/* The "DPC" algorithm sets defective pixel correction strength. */

struct DpcStatus {
	int strength; /* 0 = "off", 1 = "normal", 2 = "strong" */
};
