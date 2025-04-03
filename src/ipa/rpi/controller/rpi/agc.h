/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * AGC/AEC control algorithm
 */
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "../agc_algorithm.h"

#include "agc_channel.h"

namespace RPiController {

struct AgcChannelData {
	AgcChannel channel;
	std::optional<DeviceStatus> deviceStatus;
	StatisticsPtr statistics;
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
	void setFlickerPeriod(libcamera::utils::Duration flickerPeriod) override;
	void setMaxExposureTime(libcamera::utils::Duration maxExposureTime) override;
	void setFixedExposureTime(unsigned int channelIndex,
				  libcamera::utils::Duration fixedExposureTime) override;
	void setFixedAnalogueGain(unsigned int channelIndex,
				  double fixedAnalogueGain) override;
	void setMeteringMode(std::string const &meteringModeName) override;
	void setExposureMode(std::string const &exposureModeName) override;
	void setConstraintMode(std::string const &contraintModeName) override;
	void enableAutoExposure() override;
	void disableAutoExposure() override;
	bool autoExposureEnabled() const override;
	void enableAutoGain() override;
	void disableAutoGain() override;
	bool autoGainEnabled() const override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	void setActiveChannels(const std::vector<unsigned int> &activeChannels) override;

private:
	int checkChannel(unsigned int channel) const;
	std::vector<AgcChannelData> channelData_;
	std::vector<unsigned int> activeChannels_;
	unsigned int index_; /* index into the activeChannels_ */
	AgcChannelTotalExposures channelTotalExposures_;
};

} /* namespace RPiController */
