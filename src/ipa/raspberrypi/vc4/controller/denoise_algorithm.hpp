/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * denoise.hpp - Denoise control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPiController {

enum class DenoiseMode { Off, ColourOff, ColourFast, ColourHighQuality };

class DenoiseAlgorithm : public VC4Algorithm
{
public:
	DenoiseAlgorithm(Controller *controller) : VC4Algorithm(controller) {}
	// A Denoise algorithm must provide the following:
	virtual void SetMode(DenoiseMode mode) = 0;
};

} // namespace RPiController
