/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * sync_status.h - Sync algorithm params and status structures
 */
#pragma once

#include <libcamera/base/utils.h>

struct SyncParams {
	/* Wall clock time for this frame */
	uint64_t wallClock;
	/* Kernel timestamp for this frame */
	uint64_t sensorTimestamp;
};

struct SyncStatus {
	/* Frame length correction to apply */
	libcamera::utils::Duration frameDurationOffset;
	/* Whether the "ready time" has been reached */
	bool ready;
	/* Lag between camera frame and the "ready time" */
	int64_t lag;
	/* Whether lag is known (client has to wait for a server message) */
	bool lagKnown;
};
