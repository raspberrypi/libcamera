/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * denoise.hpp - Denoise (spatial, colour, temporal) control algorithm
 */
#pragma once

#include <map>
#include <string>

#include "algorithm.h"
#include "denoise_algorithm.h"

namespace RPiController {

// Algorithm to calculate correct denoise settings.

struct DenoiseConfig {
	double sdnDeviation;
	double sdnStrength;
	double sdnDeviation2;
	double sdnDeviationNoTdn;
	double sdnStrengthNoTdn;
	double sdnTdnBackoff;
	double cdnDeviation;
	double cdnStrength;
	double tdnDeviation;
	double tdnThreshold;
	bool tdnEnable;
	bool sdnEnable;
	bool cdnEnable;
	int read(const libcamera::YamlObject &params);
};

class Denoise : public DenoiseAlgorithm
{
public:
	Denoise(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void setMode(DenoiseMode mode) override;
	void setConfig(std::string const &name) override;

private:
	std::map<std::string, DenoiseConfig> configs_;
	DenoiseConfig *currentConfig_;
	DenoiseMode mode_;

	/* SDN parameters attenuate over time if TDN is running. */
	double currentSdnDeviation_;
	double currentSdnStrength_;
	double currentSdnDeviation2_;
};

} // namespace RPiController
