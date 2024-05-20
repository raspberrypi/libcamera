/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * contrast (gamma) control algorithm interface
 */
#pragma once

#include "algorithm.h"

namespace RPiController {

class ContrastAlgorithm : public Algorithm
{
public:
	ContrastAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* A contrast algorithm must provide the following: */
	virtual void setBrightness(double brightness) = 0;
	virtual void setContrast(double contrast) = 0;
	virtual void enableCe(bool enable) = 0;
	virtual void restoreCe() = 0;
};

} /* namespace RPiController */
