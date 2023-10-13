/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * hdr.cpp - HDR control algorithm
 */

#include "hdr.h"

#include <libcamera/base/log.h>

#include "../agc_status.h"
#include "../stitch_status.h"
#include "../tonemap_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiHdr)

#define NAME "rpi.hdr"

void HdrConfig::read(const libcamera::YamlObject &params, const std::string &modeName)
{
	name = modeName;

	if (!params.contains("cadence"))
		LOG(RPiHdr, Fatal) << "No cadence for HDR mode " << name;
	cadence = params["cadence"].getList<unsigned int>().value();
	if (cadence.empty())
		LOG(RPiHdr, Fatal) << "Empty cadence in HDR mode " << name;

	/*
	 * In the JSON file it's easier to use the channel name as the key, but
	 * for us it's convenient to swap them over.
	 */
	for (const auto &[k, v] : params["channel_map"].asDict())
		channelMap[v.get<unsigned int>().value()] = k;

	/* Read any tonemap parameters. */
	tonemapEnable = params["tonemap_enable"].get<int>(0);
	detailConstant = params["detail_constant"].get<uint16_t>(50);
	detailSlope = params["detail_slope"].get<double>(8.0);
	iirStrength = params["iir_strength"].get<double>(8.0);
	strength = params["strength"].get<double>(1.5);

	if (tonemapEnable) {
		/* We need either an explicit tonemap, or the information to build them dynamically. */
		if (params.contains("tonemap")) {
			if (tonemap.read(params["tonemap"]))
				LOG(RPiHdr, Fatal) << "Failed to read tonemap in HDR mode " << name;
		} else {
			if (target.read(params["target"]))
				LOG(RPiHdr, Fatal) << "Failed to read target in HDR mode " << name;
			if (maxSlope.read(params["max_slope"]))
				LOG(RPiHdr, Fatal) << "Failed to read max_slope in HDR mode " << name;
			minSlope = params["min_slope"].get<double>(1.0);
			maxGain = params["max_gain"].get<double>(64.0);
			step = params["step"].get<double>(0.05);
			speed = params["speed"].get<double>(0.5);
		}
	}

	/* Read any stitch parameters. */
	stitchEnable = params["stitch_enable"].get<int>(0);
	thresholdLo = params["threshold_lo"].get<uint16_t>(50000);
	motionThreshold = params["motion_threshold"].get<double>(0.005);
	diffPower = params["diff_power"].get<uint8_t>(13);
	if (diffPower > 15)
		LOG(RPiHdr, Fatal) << "Bad diff_power value in HDR mode " << name;
}

Hdr::Hdr(Controller *controller)
	: HdrAlgorithm(controller)
{
}

char const *Hdr::name() const
{
	return NAME;
}

int Hdr::read(const libcamera::YamlObject &params)
{
	/* Make an "HDR off" mode by default so that tuning files don't have to. */
	HdrConfig &offMode = config_["Off"];
	offMode.name = "Off";
	offMode.cadence = { 0 };
	offMode.channelMap[0] = "None";
	status_.mode = offMode.name;
	delayedStatus_.mode = offMode.name;

	/*
	 * But we still allow the tuning file to override the "Off" mode if it wants.
	 * For example, maybe an application will make channel 0 be the "short"
	 * channel, in order to apply other AGC controls to it.
	 */
	for (const auto &[key, value] : params.asDict())
		config_[key].read(value, key);

	return 0;
}

int Hdr::setMode(std::string const &mode)
{
	/* Always validate the mode, so it can be used later without checking. */
	auto it = config_.find(mode);
	if (it == config_.end()) {
		LOG(RPiHdr, Warning) << "No such HDR mode " << mode;
		return -1;
	}

	status_.mode = it->second.name;

	return 0;
}

std::vector<unsigned int> Hdr::getChannels() const
{
	return config_.at(status_.mode).cadence;
}

void Hdr::updateAgcStatus(Metadata *metadata)
{
	std::scoped_lock lock(*metadata);
	AgcStatus *agcStatus = metadata->getLocked<AgcStatus>("agc.status");
	if (agcStatus) {
		HdrConfig &hdrConfig = config_[status_.mode];
		auto it = hdrConfig.channelMap.find(agcStatus->channel);
		if (it != hdrConfig.channelMap.end()) {
			status_.channel = it->second;
			agcStatus->hdr = status_;
		} else
			LOG(RPiHdr, Warning) << "Channel " << agcStatus->channel
					     << " not found in mode " << status_.mode;
	} else
		LOG(RPiHdr, Warning) << "No agc.status found";
}

void Hdr::switchMode([[maybe_unused]] CameraMode const &cameraMode, Metadata *metadata)
{
	updateAgcStatus(metadata);
	delayedStatus_ = status_;
}

bool Hdr::updateTonemap(StatisticsPtr &stats, HdrConfig &config)
{
	/* When there's a change of HDR mode we start over with a new tonemap curve. */
	if (delayedStatus_.mode != previousMode_) {
		previousMode_ = delayedStatus_.mode;
		tonemap_ = Pwl();
	}

	/* No tonemapping. No need to output a tonemap.status. */
	if (!config.tonemapEnable)
		return false;

	/* If an explicit tonemap was given, use it. */
	if (!config.tonemap.empty()) {
		tonemap_ = config.tonemap;
		return true;
	}

	/*
	 * We only update the tonemap on short frames when in multi-exposure mode. But
	 * we still need to output the most recent tonemap. Possibly we should make the
	 * config indicate the channels for which we should update the tonemap?
	 */
	if (delayedStatus_.mode == "MultiExposure" && delayedStatus_.channel != "short")
		return true;

	/* Build the tonemap dynamically using the image histogram. */
	Pwl tonemap;
	tonemap.append(0, 0);

	double prev_input_val = 0;
	double prev_output_val = 0;
	const double step2 = config.step / 2;
	for (double q = config.step; q < 1.0 - step2; q += config.step) {
		double q_lo = std::max(0.0, q - step2);
		double q_hi = std::min(1.0, q + step2);
		double iqm = stats->yHist.interQuantileMean(q_lo, q_hi);
		double input_val = std::min(iqm * 64, 65535.0);

		if (input_val > prev_input_val + 1) {
			/* We're going to calcualte a Pwl to map input_val to this output_val. */
			double want_output_val = config.target.eval(q) * 65535;
			/* But we must ensure we aren't applying too small or too great a local gain. */
			double want_slope = (want_output_val - prev_output_val) / (input_val - prev_input_val);
			double slope = std::clamp(want_slope, config.minSlope,
						  config.maxSlope.eval(q));
			double output_val = prev_output_val + slope * (input_val - prev_input_val);
			output_val = std::min(output_val, config.maxGain * input_val);
			output_val = std::clamp(output_val, 0.0, 65535.0);
			/* Let the tonemap adapte slightly more gently from frame to frame. */
			if (!tonemap_.empty()) {
				double old_output_val = tonemap_.eval(input_val);
				output_val = config.speed * output_val +
					     (1 - config.speed) * old_output_val;
			}
			LOG(RPiHdr, Debug) << "q " << q << " input " << input_val
					   << " output " << want_output_val << " slope " << want_slope
					   << " slope " << slope << " output " << output_val;
			tonemap.append(input_val, output_val);
			prev_input_val = input_val;
			prev_output_val = output_val;
		}
	}

	tonemap.append(65535, 65535);
	/* tonemap.debug(); */
	tonemap_ = tonemap;

	return true;
}

void Hdr::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	/* Note what HDR channel this frame will be once it comes back to us. */
	updateAgcStatus(imageMetadata);

	/*
	 * Now figure out what HDR channel this frame is. It should be available in the
	 * agc.delayed_status, unless this is an early frame after a mode switch, in which
	 * case delayedStatus_ should be right.
	 */
	AgcStatus agcStatus;
	if (!imageMetadata->get<AgcStatus>("agc.delayed_status", agcStatus))
		delayedStatus_ = agcStatus.hdr;

	auto it = config_.find(delayedStatus_.mode);
	if (it == config_.end()) {
		/* Shouldn't be possible. There would be nothing we could do. */
		LOG(RPiHdr, Warning) << "Unexpected HDR mode " << delayedStatus_.mode;
		return;
	}

	HdrConfig &config = it->second;

	if (updateTonemap(stats, config)) {
		/* Add tonemap.status metadata. */
		TonemapStatus tonemapStatus;

		tonemapStatus.detailConstant = config.detailConstant;
		tonemapStatus.detailSlope = config.detailSlope;
		tonemapStatus.iirStrength = config.iirStrength;
		tonemapStatus.strength = config.strength;
		tonemapStatus.tonemap = tonemap_;

		imageMetadata->set("tonemap.status", tonemapStatus);
	}

	if (config.stitchEnable) {
		/* Add stitch.status metadata. */
		StitchStatus stitchStatus;

		stitchStatus.diffPower = config.diffPower;
		stitchStatus.motionThreshold = config.motionThreshold;
		stitchStatus.thresholdLo = config.thresholdLo;

		imageMetadata->set("stitch.status", stitchStatus);
	}
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Hdr(controller);
}
static RegisterAlgorithm reg(NAME, &create);
