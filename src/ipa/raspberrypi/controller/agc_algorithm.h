/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * agc_algorithm.h - AGC/AEC control algorithm interface
 */
#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace RPiController {

class AgcAlgorithm : public Algorithm
{
public:
	AgcAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* An AGC algorithm must provide the following: */
	virtual unsigned int getConvergenceFrames() const = 0;
	virtual void setEv(double ev) = 0;
	virtual void setFlickerPeriod(libcamera::utils::Duration flickerPeriod) = 0;
	virtual void setFixedShutter(libcamera::utils::Duration fixedShutter) = 0;
	virtual void setMaxShutter(libcamera::utils::Duration maxShutter) = 0;
	virtual void setFixedAnalogueGain(double fixedAnalogueGain) = 0;
	virtual void setMeteringMode(std::string const &meteringModeName) = 0;
	virtual void setExposureMode(std::string const &exposureModeName) = 0;
	virtual void setConstraintMode(std::string const &contraintModeName) = 0;
	virtual void enableAuto() = 0;
	virtual void disableAuto() = 0;
};

} /* namespace RPiController */
