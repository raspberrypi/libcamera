/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * sharpen_algorithm.h - sharpness control algorithm interface
 */
#pragma once

#include "algorithm.h"

namespace RPiController {

class SharpenAlgorithm : public Algorithm
{
public:
	SharpenAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* A sharpness control algorithm must provide the following: */
	virtual void setStrength(double strength) = 0;
};

} /* namespace RPiController */
