/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * HDR control algorithm
 */
#pragma once

#include <map>
#include <string>
#include <vector>

#include <libcamera/geometry.h>

#include <libipa/pwl.h>

#include "../hdr_algorithm.h"
#include "../hdr_status.h"

/* This is our implementation of an HDR algorithm. */

namespace RPiController {

struct HdrConfig {
	std::string name;
	std::vector<unsigned int> cadence;
	std::map<unsigned int, std::string> channelMap;

	/* Lens shading related parameters. */
	libcamera::ipa::Pwl spatialGainCurve; /* Brightness to gain curve for different image regions. */
	unsigned int diffusion; /* How much to diffuse the gain spatially. */

	/* Tonemap related parameters. */
	bool tonemapEnable;
	uint16_t detailConstant;
	double detailSlope;
	double iirStrength;
	double strength;
	libcamera::ipa::Pwl tonemap;
	/* These relate to adaptive tonemap calculation. */
	double speed;
	std::vector<double> hiQuantileTargets; /* quantiles to check for unsaturated images */
	double hiQuantileMaxGain; /* the max gain we'll apply when unsaturated */
	std::vector<double> quantileTargets; /* target values for histogram quantiles */
	double powerMin; /* minimum tonemap power */
	double powerMax; /* maximum tonemap power */
	std::vector<double> contrastAdjustments; /* any contrast adjustment factors */

	/* Stitch related parameters. */
	bool stitchEnable;
	uint16_t thresholdLo;
	uint8_t diffPower;
	double motionThreshold;

	void read(const libcamera::YamlObject &params, const std::string &name);
};

class Hdr : public HdrAlgorithm
{
public:
	Hdr(Controller *controller);
	char const *name() const override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	int read(const libcamera::YamlObject &params) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	int setMode(std::string const &mode) override;
	std::vector<unsigned int> getChannels() const override;

private:
	void updateAgcStatus(Metadata *metadata);
	void updateGains(StatisticsPtr &stats, HdrConfig &config);
	bool updateTonemap(StatisticsPtr &stats, HdrConfig &config);

	std::map<std::string, HdrConfig> config_;
	HdrStatus status_; /* track the current HDR mode and channel */
	HdrStatus delayedStatus_; /* track the delayed HDR mode and channel */
	std::string previousMode_;
	libcamera::ipa::Pwl tonemap_;
	libcamera::Size regions_; /* stats regions */
	unsigned int numRegions_; /* total number of stats regions */
	std::vector<double> gains_[2];
};

} /* namespace RPiController */
