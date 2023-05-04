/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * hdr.h - HDR control algorithm
 */
#pragma once

#include <map>
#include <string>
#include <vector>

#include "../hdr_algorithm.h"
#include "../hdr_status.h"
#include "../pwl.h"

/* This is our implementation of an HDR algorithm. */

namespace RPiController {

struct HdrConfig {
	std::string name;
	std::vector<unsigned int> cadence;
	std::map<unsigned int, std::string> channelMap;

	/* Tonemap related parameters. */
	bool tonemapEnable;
	uint16_t detailConstant;
	double detailSlope;
	double iirStrength;
	double strength;
	/* We must have either an explicit tonemap curve, or the other parameters. */
	Pwl tonemap;
	Pwl target; /* maps histogram quatile to desired target output value */
	Pwl maxSlope; /* the maximum slope allowed at each point in the mapping */
	double minSlope; /* the minimum allowed slope */
	double maxGain; /* limit to the max absolute gain */
	double step; /* the histogram granularity for building the mapping */
	double speed; /* rate at which tonemap is updated */

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
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	int setMode(std::string const &mode) override;
	std::vector<unsigned int> getChannels() const override;

private:
	void updateAgcStatus(Metadata *metadata);
	bool updateTonemap(StatisticsPtr &stats, HdrConfig &config);

	std::map<std::string, HdrConfig> config_;
	HdrStatus status_; /* track the current HDR mode and channel */
	HdrStatus delayedStatus_; /* track the delayed HDR mode and channel */
	std::string previousMode_;
	Pwl tonemap_;
};

} /* namespace RPiController */
