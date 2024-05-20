/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * Denoise control algorithm interface
 */
#pragma once

#include <string>

#include "algorithm.h"

namespace RPiController {

enum class DenoiseMode { Off, ColourOff, ColourFast, ColourHighQuality };

class DenoiseAlgorithm : public Algorithm
{
public:
	DenoiseAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* A Denoise algorithm must provide the following: */
	virtual void setMode(DenoiseMode mode) = 0;
	/* Some platforms may not be able to define this, so supply a default. */
	virtual void setConfig([[maybe_unused]] std::string const &name) {}
};

} /* namespace RPiController */
