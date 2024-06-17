/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * AF control algorithm status
 */
#pragma once

#include <optional>

/*
 * The AF algorithm should post the following structure into the image's
 * "af.status" metadata. lensSetting should control the lens.
 */

enum class AfState {
	Idle = 0,
	Scanning,
	Focused,
	Failed
};

enum class AfPauseState {
	Running = 0,
	Pausing,
	Paused
};

struct AfStatus {
	/* state for reporting */
	AfState state;
	AfPauseState pauseState;
	/* lensSetting should be sent to the lens driver, when valid */
	std::optional<int> lensSetting;
};
