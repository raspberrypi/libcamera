/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * AGC/AEC control algorithm
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
	  activeChannels_({ 0 }), index_(0)
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
		channelTotalExposures_.resize(1, 0s);
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

	channelTotalExposures_.resize(channelData_.size(), 0s);

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

void Agc::disableAutoExposure()
{
	LOG(RPiAgc, Debug) << "disableAutoExposure";

	/* All channels are enabled/disabled together. */
	for (auto &data : channelData_)
		data.channel.disableAutoExposure();
}

void Agc::enableAutoExposure()
{
	LOG(RPiAgc, Debug) << "enableAutoExposure";

	/* All channels are enabled/disabled together. */
	for (auto &data : channelData_)
		data.channel.enableAutoExposure();
}

bool Agc::autoExposureEnabled() const
{
	LOG(RPiAgc, Debug) << "autoExposureEnabled";

	/*
	 * We always have at least one channel, and since all channels are
	 * enabled and disabled together we can simply check the first one.
	 */
	return channelData_[0].channel.autoExposureEnabled();
}

void Agc::disableAutoGain()
{
	LOG(RPiAgc, Debug) << "disableAutoGain";

	/* All channels are enabled/disabled together. */
	for (auto &data : channelData_)
		data.channel.disableAutoGain();
}

void Agc::enableAutoGain()
{
	LOG(RPiAgc, Debug) << "enableAutoGain";

	/* All channels are enabled/disabled together. */
	for (auto &data : channelData_)
		data.channel.enableAutoGain();
}

bool Agc::autoGainEnabled() const
{
	LOG(RPiAgc, Debug) << "autoGainEnabled";

	/*
	 * We always have at least one channel, and since all channels are
	 * enabled and disabled together we can simply check the first one.
	 */
	return channelData_[0].channel.autoGainEnabled();
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

void Agc::setFlickerPeriod(Duration flickerPeriod)
{
	LOG(RPiAgc, Debug) << "setFlickerPeriod " << flickerPeriod;

	/* Flicker period will be the same across all channels. */
	for (auto &data : channelData_)
		data.channel.setFlickerPeriod(flickerPeriod);
}

void Agc::setMaxExposureTime(Duration maxExposureTime)
{
	/* Frame durations will be the same across all channels too. */
	for (auto &data : channelData_)
		data.channel.setMaxExposureTime(maxExposureTime);
}

void Agc::setFixedExposureTime(unsigned int channelIndex, Duration fixedExposureTime)
{
	if (checkChannel(channelIndex))
		return;

	LOG(RPiAgc, Debug) << "setFixedExposureTime " << fixedExposureTime
			   << " for channel " << channelIndex;
	channelData_[channelIndex].channel.setFixedExposureTime(fixedExposureTime);
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

void Agc::setExposureMode(std::string const &exposureModeName)
{
	LOG(RPiAgc, Debug) << "setExposureMode " << exposureModeName;

	/* Exposure mode will be the same across all channels. */
	for (auto &data : channelData_)
		data.channel.setExposureMode(exposureModeName);
}

void Agc::setConstraintMode(std::string const &constraintModeName)
{
	LOG(RPiAgc, Debug) << "setConstraintMode " << constraintModeName;

	/* Constraint mode will be the same across all channels. */
	for (auto &data : channelData_)
		data.channel.setConstraintMode(constraintModeName);
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
	index_ = 0;
}

void Agc::switchMode(CameraMode const &cameraMode,
		     Metadata *metadata)
{
	/*
	 * We run switchMode on every channel, and then we're going to start over
	 * with the first active channel again which means that this channel's
	 * status needs to be the one we leave in the metadata.
	 */
	AgcStatus status;

	for (unsigned int channelIndex = 0; channelIndex < channelData_.size(); channelIndex++) {
		LOG(RPiAgc, Debug) << "switchMode for channel " << channelIndex;
		channelData_[channelIndex].channel.switchMode(cameraMode, metadata);
		if (channelIndex == activeChannels_[0])
			metadata->get("agc.status", status);
	}

	status.channel = activeChannels_[0];
	metadata->set("agc.status", status);
	index_ = 0;
}

static void getDelayedChannelIndex(Metadata *metadata, const char *message, unsigned int &channelIndex)
{
	std::unique_lock<RPiController::Metadata> lock(*metadata);
	AgcStatus *status = metadata->getLocked<AgcStatus>("agc.delayed_status");
	if (status)
		channelIndex = status->channel;
	else {
		/* This does happen at startup, otherwise it would be a Warning or Error. */
		LOG(RPiAgc, Debug) << message;
	}
}

static libcamera::utils::Duration
setCurrentChannelIndexGetExposure(Metadata *metadata, const char *message, unsigned int channelIndex)
{
	std::unique_lock<RPiController::Metadata> lock(*metadata);
	AgcStatus *status = metadata->getLocked<AgcStatus>("agc.status");
	libcamera::utils::Duration dur = 0s;

	if (status) {
		status->channel = channelIndex;
		dur = status->totalExposureValue;
	} else {
		/* This does happen at startup, otherwise it would be a Warning or Error. */
		LOG(RPiAgc, Debug) << message;
	}

	return dur;
}

void Agc::prepare(Metadata *imageMetadata)
{
	/*
	 * The DeviceStatus in the metadata should be correct for the image we
	 * are processing. The delayed status should tell us what channel this frame
	 * was from, so we will use that channel's prepare method.
	 *
	 * \todo To be honest, there's not much that's stateful in the prepare methods
	 * so we should perhaps re-evaluate whether prepare even needs to be done
	 * "per channel".
	 */
	unsigned int channelIndex = activeChannels_[0];
	getDelayedChannelIndex(imageMetadata, "prepare: no delayed status", channelIndex);

	LOG(RPiAgc, Debug) << "prepare for channel " << channelIndex;
	channelData_[channelIndex].channel.prepare(imageMetadata);
}

void Agc::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	/*
	 * We want to generate values for the next channel in round robin fashion
	 * (i.e. the channel at location index_ in the activeChannel list), even though
	 * the statistics we have will be for a different channel (which we find
	 * again from the delayed status).
	 */

	/* Generate updated AGC values for channel for new channel that we are requesting. */
	unsigned int channelIndex = activeChannels_[index_];
	AgcChannelData &channelData = channelData_[channelIndex];
	/* The stats that arrived with this image correspond to the following channel. */
	unsigned int statsIndex = 0;
	getDelayedChannelIndex(imageMetadata, "process: no delayed status for stats", statsIndex);
	LOG(RPiAgc, Debug) << "process for channel " << channelIndex;

	/*
	 * We keep a cache of the most recent DeviceStatus and stats for each channel,
	 * so that we can invoke the next channel's process method with the most up to date
	 * values.
	 */
	LOG(RPiAgc, Debug) << "Save DeviceStatus and stats for channel " << statsIndex;
	DeviceStatus deviceStatus;
	if (imageMetadata->get<DeviceStatus>("device.status", deviceStatus) == 0)
		channelData_[statsIndex].deviceStatus = deviceStatus;
	else
		/* Every frame should have a DeviceStatus. */
		LOG(RPiAgc, Error) << "process: no device status found";
	channelData_[statsIndex].statistics = stats;

	/*
	 * Finally fetch the most recent DeviceStatus and stats for the new channel, if both
	 * exist, and call process(). We must make the agc.status metadata record correctly
	 * which channel this is.
	 */
	StatisticsPtr *statsPtr = &stats;
	if (channelData.statistics && channelData.deviceStatus) {
		deviceStatus = *channelData.deviceStatus;
		statsPtr = &channelData.statistics;
	} else {
		/* Can also happen when new channels start. */
		LOG(RPiAgc, Debug) << "process: channel " << channelIndex << " not seen yet";
	}

	channelData.channel.process(*statsPtr, deviceStatus, imageMetadata, channelTotalExposures_);
	auto dur = setCurrentChannelIndexGetExposure(imageMetadata, "process: no AGC status found",
						     channelIndex);
	if (dur)
		channelTotalExposures_[channelIndex] = dur;

	/* And onto the next channel for the next call. */
	index_ = (index_ + 1) % activeChannels_.size();
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Agc(controller);
}
static RegisterAlgorithm reg(NAME, &create);
