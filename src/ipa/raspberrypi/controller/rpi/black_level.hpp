/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * black_level.hpp - black level control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../black_level_status.h"

// This is our implementation of the "black level algorithm".

namespace RPi {

class BlackLevel : public Algorithm
{
public:
	BlackLevel(Controller *controller);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;

private:
	double black_level_r_;
	double black_level_g_;
	double black_level_b_;
};

} // namespace RPi
