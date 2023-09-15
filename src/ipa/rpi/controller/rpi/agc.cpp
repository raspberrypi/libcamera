/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * agc.cpp - AGC/AEC control algorithm
 */

#include "agc.h"

#include <libcamera/base/log.h>

#include "../metadata.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(RPiAgc)

#define NAME "rpi.agc"

Agc::Agc(Controller *controller)
	: AgcAlgorithm(controller),
	  activeChannels_({ 0 })
{
}

char const *Agc::name() const
{
	return NAME;
}

int Agc::read(const libcamera::YamlObject &params)
{
	/*
	 * When there is only a single channel we can read the old style syntax.
	 * Otherwise we expect a "channels" keyword followed by a list of configurations.
	 */
	if (!params.contains("channels")) {
		LOG(RPiAgc, Debug) << "Single channel only";
		channelData_.emplace_back();
		return channelData_.back().channel.read(params, getHardwareConfig());
	}

	const auto &channels = params["channels"].asList();
	for (auto ch = channels.begin(); ch != channels.end(); ch++) {
		LOG(RPiAgc, Debug) << "Read AGC channel";
		channelData_.emplace_back();
		int ret = channelData_.back().channel.read(*ch, getHardwareConfig());
		if (ret)
			return ret;
	}

	LOG(RPiAgc, Debug) << "Read " << channelData_.size() << " channel(s)";
	if (channelData_.empty()) {
		LOG(RPiAgc, Error) << "No AGC channels provided";
		return -1;
	}

	return 0;
}

int Agc::checkChannel(unsigned int channelIndex) const
{
	if (channelIndex >= channelData_.size()) {
		LOG(RPiAgc, Warning) << "AGC channel " << channelIndex << " not available";
		return -1;
	}

	return 0;
}

void Agc::disableAuto(unsigned int channelIndex)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "disableAuto for channel " << channelIndex;
	channelData_[channelIndex].channel.disableAuto();
}

void Agc::enableAuto(unsigned int channelIndex)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "enableAuto for channel " << channelIndex;
	channelData_[channelIndex].channel.enableAuto();
}

unsigned int Agc::getConvergenceFrames() const
{
	/* If there are n channels, it presumably takes n times as long to converge. */
	return channelData_[0].channel.getConvergenceFrames() * activeChannels_.size();
}

std::vector<double> const &Agc::getWeights() const
{
	/*
	 * In future the metering weights may be determined differently, making it
	 * difficult to associate different sets of weight with different channels.
	 * Therefore we shall impose a limitation, at least for now, that all
	 * channels will use the same weights.
	 */
	return channelData_[0].channel.getWeights();
}

void Agc::setEv(unsigned int channelIndex, double ev)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "setEv " << ev << " for channel " << channelIndex;
	channelData_[channelIndex].channel.setEv(ev);
}

void Agc::setFlickerPeriod(unsigned int channelIndex, Duration flickerPeriod)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "setFlickerPeriod " << flickerPeriod
			   << " for channel " << channelIndex;
	channelData_[channelIndex].channel.setFlickerPeriod(flickerPeriod);
}

void Agc::setMaxShutter(Duration maxShutter)
{
	/* Frame durations will be the same across all channels too. */
	for (auto &data : channelData_)
		data.channel.setMaxShutter(maxShutter);
}

void Agc::setFixedShutter(unsigned int channelIndex, Duration fixedShutter)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "setFixedShutter " << fixedShutter
			   << " for channel " << channelIndex;
	channelData_[channelIndex].channel.setFixedShutter(fixedShutter);
}

void Agc::setFixedAnalogueGain(unsigned int channelIndex, double fixedAnalogueGain)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "setFixedAnalogueGain " << fixedAnalogueGain
			   << " for channel " << channelIndex;
	channelData_[channelIndex].channel.setFixedAnalogueGain(fixedAnalogueGain);
}

void Agc::setMeteringMode(std::string const &meteringModeName)
{
	/* Metering modes will be the same across all channels too. */
	for (auto &data : channelData_)
		data.channel.setMeteringMode(meteringModeName);
}

void Agc::setExposureMode(unsigned int channelIndex, std::string const &exposureModeName)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "setExposureMode " << exposureModeName
			   << " for channel " << channelIndex;
	channelData_[channelIndex].channel.setExposureMode(exposureModeName);
}

void Agc::setConstraintMode(unsigned int channelIndex, std::string const &constraintModeName)
{
	if (checkChannel(channelIndex))
		return;

	channelData_[channelIndex].channel.setConstraintMode(constraintModeName);
}

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v)
{
	os << "{";
	for (const auto &e : v)
		os << " " << e;
	os << " }";
	return os;
}

void Agc::setActiveChannels(const std::vector<unsigned int> &activeChannels)
{
	if (activeChannels.empty()) {
		LOG(RPiAgc, Warning) << "No active AGC channels supplied";
		return;
	}

	for (auto index : activeChannels)
		if (checkChannel(index))
			return;

	LOG(RPiAgc, Debug) << "setActiveChannels " << activeChannels;
	activeChannels_ = activeChannels;
}

void Agc::switchMode(CameraMode const &cameraMode,
		     Metadata *metadata)
{
	LOG(RPiAgc, Debug) << "switchMode for channel 0";
	channelData_[0].channel.switchMode(cameraMode, metadata);
}

void Agc::prepare(Metadata *imageMetadata)
{
	LOG(RPiAgc, Debug) << "prepare for channel 0";
	channelData_[0].channel.prepare(imageMetadata);
}

void Agc::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	LOG(RPiAgc, Debug) << "process for channel 0";
	channelData_[0].channel.process(stats, imageMetadata);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Agc(controller);
}
static RegisterAlgorithm reg(NAME, &create);
