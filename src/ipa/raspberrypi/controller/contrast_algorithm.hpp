/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * contrast_algorithm.hpp - contrast (gamma) control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPiController {

class ContrastAlgorithm : public Algorithm
{
public:
	ContrastAlgorithm(Controller *controller) : Algorithm(controller) {}
	// A contrast algorithm must provide the following:
	virtual void setBrightness(double brightness) = 0;
	virtual void setContrast(double contrast) = 0;
};

} // namespace RPiController
