/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * black_level.h - black level control algorithm
 */
#pragma once

#include "../algorithm.h"
#include "../black_level_status.h"

/* This is our implementation of the "black level algorithm". */

namespace RPiController {

class BlackLevel : public Algorithm
{
public:
	BlackLevel(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	double blackLevelR_;
	double blackLevelG_;
	double blackLevelB_;
};

} /* namespace RPiController */
