/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * HDR control algorithm interface
 */
#pragma once

#include <vector>

#include "algorithm.h"

namespace RPiController {

class HdrAlgorithm : public Algorithm
{
public:
	HdrAlgorithm(Controller *controller)
		: Algorithm(controller) {}
	/* An HDR algorithm must provide the following: */
	virtual int setMode(std::string const &modeName) = 0;
	virtual std::vector<unsigned int> getChannels() const = 0;
};

} /* namespace RPiController */
