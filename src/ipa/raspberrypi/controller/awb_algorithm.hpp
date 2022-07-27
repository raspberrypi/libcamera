/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb_algorithm.hpp - AWB control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPiController {

class AwbAlgorithm : public Algorithm
{
public:
	AwbAlgorithm(Controller *controller) : Algorithm(controller) {}
	// An AWB algorithm must provide the following:
	virtual unsigned int getConvergenceFrames() const = 0;
	virtual void setMode(std::string const &modeName) = 0;
	virtual void setManualGains(double manualR, double manualB) = 0;
};

} // namespace RPiController
