/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * lux.hpp - Lux control algorithm
 */
#pragma once

#include <atomic>
#include <mutex>

#include "../lux_status.h"
#include "../algorithm.hpp"

// This is our implementation of the "lux control algorithm".

namespace RPi {

class Lux : public Algorithm
{
public:
	Lux(Controller *controller);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;
	void SetCurrentAperture(double aperture);

private:
	// These values define the conditions of the reference image, against
	// which we compare the new image.
	double reference_shutter_speed_; // in micro-seconds
	double reference_gain_;
	double reference_aperture_; // units of 1/f
	double reference_Y_; // out of 65536
	double reference_lux_;
	std::atomic<double> current_aperture_;
	LuxStatus status_;
	std::mutex mutex_;
};

} // namespace RPi
