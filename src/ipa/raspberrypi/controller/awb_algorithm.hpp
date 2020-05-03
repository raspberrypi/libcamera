/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb_algorithm.hpp - AWB control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPi {

class AwbAlgorithm : public Algorithm
{
public:
	AwbAlgorithm(Controller *controller) : Algorithm(controller) {}
	// An AWB algorithm must provide the following:
	virtual void SetMode(std::string const &mode_name) = 0;
	virtual void SetManualGains(double manual_r, double manual_b) = 0;
};

} // namespace RPi
