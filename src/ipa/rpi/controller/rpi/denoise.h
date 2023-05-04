/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * denoise.hpp - Denoise (spatial, colour, temporal) control algorithm
 */
#pragma once

#include "algorithm.h"
#include "denoise_algorithm.h"

namespace RPiController {

// Algorithm to calculate correct denoise settings.

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

private:
	double sdnDeviation_;
	double sdnStrength_;
	double sdnDeviation2_;
	double sdnDeviationNoTdn_;
	double sdnStrengthNoTdn_;
	double sdnTdnBackoff_;
	double cdnDeviation_;
	double cdnStrength_;
	double tdnDeviation_;
	double tdnThreshold_;
	DenoiseMode mode_;
	bool tdnEnable_;
	bool sdnEnable_;
	bool cdnEnable_;

	/* SDN parameters attenuate over time if TDN is running. */
	double currentSdnDeviation_;
	double currentSdnStrength_;
	double currentSdnDeviation2_;
};

} // namespace RPiController
