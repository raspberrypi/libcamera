/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * contrast.h - contrast (gamma) control algorithm
 */
#pragma once

#include <mutex>

#include "../contrast_algorithm.h"
#include "../pwl.h"

namespace RPiController {

/*
 * Back End algorithm to appaly correct digital gain. Should be placed after
 * Back End AWB.
 */

struct ContrastConfig {
	bool ceEnable;
	double loHistogram;
	double loLevel;
	double loMax;
	double hiHistogram;
	double hiLevel;
	double hiMax;
	Pwl gammaCurve;
};

class Contrast : public ContrastAlgorithm
{
public:
	Contrast(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void setBrightness(double brightness) override;
	void setContrast(double contrast) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

private:
	ContrastConfig config_;
	double brightness_;
	double contrast_;
	ContrastStatus status_;
	std::mutex mutex_;
};

} /* namespace RPiController */
