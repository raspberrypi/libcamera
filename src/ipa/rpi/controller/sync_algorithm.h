/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * sync_algorithm.h - Camera sync algorithm interface
 */
#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace RPiController {

class SyncAlgorithm : public Algorithm
{
public:
	enum class Mode {
		Off,
		Server,
		Client,
	};

	SyncAlgorithm(Controller *controller)
		: Algorithm(controller) {}
	virtual void setFrameDuration(libcamera::utils::Duration frameDuration) = 0;
	virtual void setReadyFrame(unsigned int frame) = 0;
	virtual void setMode(Mode mode) = 0;
};

} /* namespace RPiController */
