/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * geq.hpp - GEQ (green equalisation) control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../geq_status.h"

namespace RPi {

// Back End algorithm to apply appropriate GEQ settings.

struct GeqConfig {
	uint16_t offset;
	double slope;
	Pwl strength; // lux to strength factor
};

class Geq : public Algorithm
{
public:
	Geq(Controller *controller);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;

private:
	GeqConfig config_;
};

} // namespace RPi
