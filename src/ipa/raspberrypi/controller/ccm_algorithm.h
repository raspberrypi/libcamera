/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ccm_algorithm.h - CCM (colour correction matrix) control algorithm interface
 */
#pragma once

#include "algorithm.h"

namespace RPiController {

class CcmAlgorithm : public Algorithm
{
public:
	CcmAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* A CCM algorithm must provide the following: */
	virtual void setSaturation(double saturation) = 0;
};

} /* namespace RPiController */
