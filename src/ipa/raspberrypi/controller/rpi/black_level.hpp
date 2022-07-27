/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * black_level.hpp - black level control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../black_level_status.h"

/* This is our implementation of the "black level algorithm". */

namespace RPiController {

class BlackLevel : public Algorithm
{
public:
	BlackLevel(Controller *controller);
	char const *name() const override;
	void read(boost::property_tree::ptree const &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	double blackLevelR_;
	double blackLevelG_;
	double blackLevelB_;
};

} /* namespace RPiController */
