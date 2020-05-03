/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sdn.hpp - SDN (spatial denoise) control algorithm
 */
#pragma once

#include "../algorithm.hpp"

namespace RPi {

// Algorithm to calculate correct spatial denoise (SDN) settings.

class Sdn : public Algorithm
{
public:
	Sdn(Controller *controller = NULL);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Initialise() override;
	void Prepare(Metadata *image_metadata) override;

private:
	double deviation_;
	double strength_;
};

} // namespace RPi
