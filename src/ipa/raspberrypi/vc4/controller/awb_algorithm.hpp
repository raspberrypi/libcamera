/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb_algorithm.hpp - AWB control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPiController {

class AwbAlgorithm : public VC4Algorithm
{
public:
	AwbAlgorithm(Controller *controller) : VC4Algorithm(controller) {}
	// An AWB algorithm must provide the following:
	virtual unsigned int GetConvergenceFrames() const = 0;
	virtual void SetMode(std::string const &mode_name) = 0;
	virtual void SetManualGains(double manual_r, double manual_b) = 0;
};

} // namespace RPiController
