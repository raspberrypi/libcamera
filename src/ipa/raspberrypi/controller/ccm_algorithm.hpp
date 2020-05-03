/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ccm_algorithm.hpp - CCM (colour correction matrix) control algorithm interface
 */
#pragma once

#include "algorithm.hpp"

namespace RPi {

class CcmAlgorithm : public Algorithm
{
public:
	CcmAlgorithm(Controller *controller) : Algorithm(controller) {}
	// A CCM algorithm must provide the following:
	virtual void SetSaturation(double saturation) = 0;
};

} // namespace RPi
