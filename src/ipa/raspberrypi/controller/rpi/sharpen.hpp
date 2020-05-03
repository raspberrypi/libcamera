/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sharpen.hpp - sharpening control algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../sharpen_status.h"

// This is our implementation of the "sharpen algorithm".

namespace RPi {

class Sharpen : public Algorithm
{
public:
	Sharpen(Controller *controller);
	char const *Name() const override;
	void SwitchMode(CameraMode const &camera_mode) override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;

private:
	double threshold_;
	double strength_;
	double limit_;
	std::atomic<double> mode_factor_;
};

} // namespace RPi
