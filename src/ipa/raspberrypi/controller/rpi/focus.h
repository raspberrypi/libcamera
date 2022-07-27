/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * focus.h - focus algorithm
 */
#pragma once

#include "../algorithm.h"
#include "../metadata.h"

/*
 * The "focus" algorithm. All it does it print out a version of the
 * focus contrast measure; there is no actual auto-focus mechanism to
 * control.
 */

namespace RPiController {

class Focus : public Algorithm
{
public:
	Focus(Controller *controller);
	char const *name() const override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
};

} /* namespace RPiController */
