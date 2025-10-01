/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * Decompand control algorithm interface
 */
#pragma once

#include "libipa/pwl.h"

#include "algorithm.h"

namespace RPiController {

class DecompandAlgorithm : public Algorithm
{
public:
	DecompandAlgorithm(Controller *controller = NULL)
		: Algorithm(controller)
	{
	}
};

} /* namespace RPiController */
