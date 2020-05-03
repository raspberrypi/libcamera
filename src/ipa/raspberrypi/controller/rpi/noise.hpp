/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * noise.hpp - Noise control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../noise_status.h"

// This is our implementation of the "noise algorithm".

namespace RPi {

class Noise : public Algorithm
{
public:
	Noise(Controller *controller);
	char const *Name() const override;
	void SwitchMode(CameraMode const &camera_mode) override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;

private:
	// the noise profile for analogue gain of 1.0
	double reference_constant_;
	double reference_slope_;
	std::atomic<double> mode_factor_;
};

} // namespace RPi
