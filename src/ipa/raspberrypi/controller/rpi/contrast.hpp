/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * contrast.hpp - contrast (gamma) control algorithm
 */
#pragma once

#include <atomic>
#include <mutex>

#include "../contrast_algorithm.hpp"
#include "../pwl.hpp"

namespace RPi {

// Back End algorithm to appaly correct digital gain. Should be placed after
// Back End AWB.

struct ContrastConfig {
	bool ce_enable;
	double lo_histogram;
	double lo_level;
	double lo_max;
	double hi_histogram;
	double hi_level;
	double hi_max;
	Pwl gamma_curve;
};

class Contrast : public ContrastAlgorithm
{
public:
	Contrast(Controller *controller = NULL);
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void SetBrightness(double brightness) override;
	void SetContrast(double contrast) override;
	void Initialise() override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;

private:
	ContrastConfig config_;
	std::atomic<double> brightness_;
	std::atomic<double> contrast_;
	ContrastStatus status_;
	std::mutex mutex_;
};

} // namespace RPi
