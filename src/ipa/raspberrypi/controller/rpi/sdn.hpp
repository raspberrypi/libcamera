/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sdn.hpp - SDN (spatial denoise) control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../denoise_algorithm.hpp"

namespace RPiController {

/* Algorithm to calculate correct spatial denoise (SDN) settings. */

class Sdn : public DenoiseAlgorithm
{
public:
	Sdn(Controller *controller = NULL);
	char const *name() const override;
	void read(boost::property_tree::ptree const &params) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;
	void setMode(DenoiseMode mode) override;

private:
	double deviation_;
	double strength_;
	DenoiseMode mode_;
};

} /* namespace RPiController */
