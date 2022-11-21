/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * awb_algorithm.h - AWB control algorithm interface
 */
#pragma once

#include "algorithm.h"

namespace RPiController {

class AwbAlgorithm : public Algorithm
{
public:
	AwbAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* An AWB algorithm must provide the following: */
	virtual unsigned int getConvergenceFrames() const = 0;
	virtual void setMode(std::string const &modeName) = 0;
	virtual void setManualGains(double manualR, double manualB) = 0;
	virtual void enableAuto() = 0;
	virtual void disableAuto() = 0;
};

} /* namespace RPiController */
