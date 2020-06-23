/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * sharpen_algorithm.hpp - sharpness control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPi {

class SharpenAlgorithm : public Algorithm
{
public:
	SharpenAlgorithm(Controller *controller) : Algorithm(controller) {}
	// A sharpness control algorithm must provide the following:
	virtual void SetStrength(double strength) = 0;
};

} // namespace RPi
