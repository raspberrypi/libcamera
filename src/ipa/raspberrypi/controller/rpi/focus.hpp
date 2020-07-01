/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * focus.hpp - focus algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../metadata.hpp"

/*
 * The "focus" algorithm. All it does it print out a version of the
 * focus contrast measure; there is no actual auto-focus mechanism to
 * control.
 */

namespace RPi {

class Focus : public Algorithm
{
public:
	Focus(Controller *controller);
	char const *Name() const override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;
};

} /* namespace RPi */
