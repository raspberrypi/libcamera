/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * agc.h - AGC/AEC control algorithm
 */
#pragma once

#include <string>
#include <vector>

#include "../agc_algorithm.h"

#include "agc_channel.h"

namespace RPiController {

struct AgcChannelData {
	AgcChannel channel;
};

class Agc : public AgcAlgorithm
{
public:
	Agc(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	unsigned int getConvergenceFrames() const override;
	std::vector<double> const &getWeights() const override;
	void setEv(unsigned int channel, double ev) override;
	void setFlickerPeriod(unsigned int channelIndex,
			      libcamera::utils::Duration flickerPeriod) override;
	void setMaxShutter(libcamera::utils::Duration maxShutter) override;
	void setFixedShutter(unsigned int channelIndex,
			     libcamera::utils::Duration fixedShutter) override;
	void setFixedAnalogueGain(unsigned int channelIndex,
				  double fixedAnalogueGain) override;
	void setMeteringMode(std::string const &meteringModeName) override;
	void setExposureMode(unsigned int channelIndex,
			     std::string const &exposureModeName) override;
	void setConstraintMode(unsigned int channelIndex,
			       std::string const &contraintModeName) override;
	void enableAuto(unsigned int channelIndex) override;
	void disableAuto(unsigned int channelIndex) override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	void setActiveChannels(const std::vector<unsigned int> &activeChannels) override;

private:
	int checkChannel(unsigned int channel) const;
	std::vector<AgcChannelData> channelData_;
	std::vector<unsigned int> activeChannels_;
};

} /* namespace RPiController */
