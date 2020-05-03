/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc_algorithm.hpp - AGC/AEC control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPi {

class AgcAlgorithm : public Algorithm
{
public:
	AgcAlgorithm(Controller *controller) : Algorithm(controller) {}
	// An AGC algorithm must provide the following:
	virtual void SetEv(double ev) = 0;
	virtual void SetFlickerPeriod(double flicker_period) = 0;
	virtual void SetFixedShutter(double fixed_shutter) = 0; // microseconds
	virtual void SetFixedAnalogueGain(double fixed_analogue_gain) = 0;
	virtual void SetMeteringMode(std::string const &metering_mode_name) = 0;
	virtual void SetExposureMode(std::string const &exposure_mode_name) = 0;
	virtual void
	SetConstraintMode(std::string const &contraint_mode_name) = 0;
};

} // namespace RPi
