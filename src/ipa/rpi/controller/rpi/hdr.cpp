/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * HDR control algorithm
 */

#include "hdr.h"

#include <cmath>

#include <libcamera/base/log.h>

#include "../agc_status.h"
#include "../alsc_status.h"
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

	/* Lens shading related parameters. */
	if (params.contains("spatial_gain_curve")) {
		spatialGainCurve = params["spatial_gain_curve"].get<ipa::Pwl>(ipa::Pwl{});
	} else if (params.contains("spatial_gain")) {
		double spatialGain = params["spatial_gain"].get<double>(2.0);
		spatialGainCurve.append(0.0, spatialGain);
		spatialGainCurve.append(0.01, spatialGain);
		spatialGainCurve.append(0.06, 1.0); /* maybe make this programmable? */
		spatialGainCurve.append(1.0, 1.0);
	}

	diffusion = params["diffusion"].get<unsigned int>(3);
	/* Clip to an arbitrary limit just to stop typos from killing the system! */
	const unsigned int MAX_DIFFUSION = 15;
	if (diffusion > MAX_DIFFUSION) {
		diffusion = MAX_DIFFUSION;
		LOG(RPiHdr, Warning) << "Diffusion value clipped to " << MAX_DIFFUSION;
	}

	/* Read any tonemap parameters. */
	tonemapEnable = params["tonemap_enable"].get<int>(0);
	detailConstant = params["detail_constant"].get<uint16_t>(0);
	detailSlope = params["detail_slope"].get<double>(0.0);
	iirStrength = params["iir_strength"].get<double>(8.0);
	strength = params["strength"].get<double>(1.5);
	if (tonemapEnable)
		tonemap = params["tonemap"].get<ipa::Pwl>(ipa::Pwl{});
	speed = params["speed"].get<double>(1.0);
	if (params.contains("hi_quantile_targets")) {
		hiQuantileTargets = params["hi_quantile_targets"].getList<double>().value();
		if (hiQuantileTargets.empty() || hiQuantileTargets.size() % 2)
			LOG(RPiHdr, Fatal) << "hi_quantile_targets much be even and non-empty";
	} else
		hiQuantileTargets = { 0.95, 0.65, 0.5, 0.28, 0.3, 0.25 };
	hiQuantileMaxGain = params["hi_quantile_max_gain"].get<double>(1.6);
	if (params.contains("quantile_targets")) {
		quantileTargets = params["quantile_targets"].getList<double>().value();
		if (quantileTargets.empty() || quantileTargets.size() % 2)
			LOG(RPiHdr, Fatal) << "quantile_targets much be even and non-empty";
	} else
		quantileTargets = { 0.2, 0.03, 1.0, 0.15 };
	powerMin = params["power_min"].get<double>(0.65);
	powerMax = params["power_max"].get<double>(1.0);
	if (params.contains("contrast_adjustments")) {
		contrastAdjustments = params["contrast_adjustments"].getList<double>().value();
	} else
		contrastAdjustments = { 0.5, 0.75 };

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
	regions_ = controller->getHardwareConfig().awbRegions;
	numRegions_ = regions_.width * regions_.height;
	gains_[0].resize(numRegions_, 1.0);
	gains_[1].resize(numRegions_, 1.0);
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

void Hdr::prepare(Metadata *imageMetadata)
{
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
	if (config.spatialGainCurve.empty())
		return;

	AlscStatus alscStatus{}; /* some compilers seem to require the braces */
	if (imageMetadata->get<AlscStatus>("alsc.status", alscStatus)) {
		LOG(RPiHdr, Warning) << "No ALSC status";
		return;
	}

	/* The final gains ended up in the odd or even array, according to diffusion. */
	std::vector<double> &gains = gains_[config.diffusion & 1];
	for (unsigned int i = 0; i < numRegions_; i++) {
		alscStatus.r[i] *= gains[i];
		alscStatus.g[i] *= gains[i];
		alscStatus.b[i] *= gains[i];
	}
	imageMetadata->set("alsc.status", alscStatus);
}

bool Hdr::updateTonemap([[maybe_unused]] StatisticsPtr &stats, HdrConfig &config)
{
	/* When there's a change of HDR mode we start over with a new tonemap curve. */
	if (delayedStatus_.mode != previousMode_) {
		previousMode_ = delayedStatus_.mode;
		tonemap_ = ipa::Pwl();
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
	 * We wouldn't update the tonemap on short frames when in multi-exposure mode. But
	 * we still need to output the most recent tonemap. Possibly we should make the
	 * config indicate the channels for which we should update the tonemap?
	 */
	if (delayedStatus_.mode == "MultiExposure" && delayedStatus_.channel != "short")
		return true;

	/*
	 * Create a tonemap dynamically. We have three ingredients.
	 *
	 * 1. We have a list of "hi quantiles" and "targets". We use these to judge if
	 * the image does seem to be reasonably saturated. If it isn't, we calculate
	 * a gain that we will feed as a linear factor into the tonemap generation.
	 * This prevents unsaturated images from beoming quite so "flat".
	 *
	 * 2. We have a list of quantile/target pairs for the bottom of the histogram.
	 * We use these to calculate how much gain we must apply to the bottom of the
	 * tonemap. We apply this gain as a power curve so as not to blow out the top
	 * end.
	 *
	 * 3. Finally, when we generate the tonemap, we have some contrast adjustments
	 * for the bottom because we know that power curves can start quite steeply and
	 * cause a washed-out look.
	 */

	/* Compute the linear gain from the headroom for saturation at the top. */
	double gain = 10; /* arbitrary, but hiQuantileMaxGain will clamp it later */
	for (unsigned int i = 0; i < config.hiQuantileTargets.size(); i += 2) {
		double quantile = config.hiQuantileTargets[i];
		double target = config.hiQuantileTargets[i + 1];
		double value = stats->yHist.interQuantileMean(quantile, 1.0) / 1024.0;
		double newGain = target / (value + 0.01);
		gain = std::min(gain, newGain);
	}
	gain = std::clamp(gain, 1.0, config.hiQuantileMaxGain);

	/* Compute the power curve from the amount of gain needed at the bottom. */
	double min_power = 2; /* arbitrary, but config.powerMax will clamp it later */
	for (unsigned int i = 0; i < config.quantileTargets.size(); i += 2) {
		double quantile = config.quantileTargets[i];
		double target = config.quantileTargets[i + 1];
		double value = stats->yHist.interQuantileMean(0, quantile) / 1024.0;
		value = std::min(value * gain, 1.0);
		double power = log(target + 1e-6) / log(value + 1e-6);
		min_power = std::min(min_power, power);
	}
	double power = std::clamp(min_power, config.powerMin, config.powerMax);

	/* Generate the tonemap, including the contrast adjustment factors. */
	libcamera::ipa::Pwl tonemap;
	tonemap.append(0, 0);
	for (unsigned int i = 0; i <= 6; i++) {
		double x = 1 << (i + 9); /* x loops from 512 to 32768 inclusive */
		double y = pow(std::min(x * gain, 65535.0) / 65536.0, power) * 65536;
		if (i < config.contrastAdjustments.size())
			y *= config.contrastAdjustments[i];
		if (!tonemap_.empty())
			y = y * config.speed + tonemap_.eval(x) * (1 - config.speed);
		tonemap.append(x, y);
	}
	tonemap.append(65535, 65535);
	tonemap_ = tonemap;

	return true;
}

static void averageGains(std::vector<double> &src, std::vector<double> &dst, const Size &size)
{
#define IDX(y, x) ((y)*size.width + (x))
	unsigned int lastCol = size.width - 1; /* index of last column */
	unsigned int preLastCol = lastCol - 1; /* and the column before that */
	unsigned int lastRow = size.height - 1; /* index of last row */
	unsigned int preLastRow = lastRow - 1; /* and the row before that */

	/* Corners first. */
	dst[IDX(0, 0)] = (src[IDX(0, 0)] + src[IDX(0, 1)] + src[IDX(1, 0)]) / 3;
	dst[IDX(0, lastCol)] = (src[IDX(0, lastCol)] + src[IDX(0, preLastCol)] + src[IDX(1, lastCol)]) / 3;
	dst[IDX(lastRow, 0)] = (src[IDX(lastRow, 0)] + src[IDX(lastRow, 1)] + src[IDX(preLastRow, 0)]) / 3;
	dst[IDX(lastRow, lastCol)] = (src[IDX(lastRow, lastCol)] + src[IDX(lastRow, preLastCol)] +
				      src[IDX(preLastRow, lastCol)]) /
				     3;

	/* Now the edges. */
	for (unsigned int i = 1; i < lastCol; i++) {
		dst[IDX(0, i)] = (src[IDX(0, i - 1)] + src[IDX(0, i)] + src[IDX(0, i + 1)] + src[IDX(1, i)]) / 4;
		dst[IDX(lastRow, i)] = (src[IDX(lastRow, i - 1)] + src[IDX(lastRow, i)] +
					src[IDX(lastRow, i + 1)] + src[IDX(preLastRow, i)]) /
				       4;
	}

	for (unsigned int i = 1; i < lastRow; i++) {
		dst[IDX(i, 0)] = (src[IDX(i - 1, 0)] + src[IDX(i, 0)] + src[IDX(i + 1, 0)] + src[IDX(i, 1)]) / 4;
		dst[IDX(i, 31)] = (src[IDX(i - 1, lastCol)] + src[IDX(i, lastCol)] +
				   src[IDX(i + 1, lastCol)] + src[IDX(i, preLastCol)]) /
				  4;
	}

	/* Finally the interior. */
	for (unsigned int j = 1; j < lastRow; j++) {
		for (unsigned int i = 1; i < lastCol; i++) {
			dst[IDX(j, i)] = (src[IDX(j - 1, i)] + src[IDX(j, i - 1)] + src[IDX(j, i)] +
					  src[IDX(j, i + 1)] + src[IDX(j + 1, i)]) /
					 5;
		}
	}
}

void Hdr::updateGains(StatisticsPtr &stats, HdrConfig &config)
{
	if (config.spatialGainCurve.empty())
		return;

	/* When alternating exposures, only compute these gains for the short frame. */
	if (delayedStatus_.mode == "MultiExposure" && delayedStatus_.channel != "short")
		return;

	for (unsigned int i = 0; i < numRegions_; i++) {
		auto &region = stats->awbRegions.get(i);
		unsigned int counted = region.counted;
		counted += (counted == 0); /* avoid div by zero */
		double r = region.val.rSum / counted;
		double g = region.val.gSum / counted;
		double b = region.val.bSum / counted;
		double brightness = std::max({ r, g, b }) / 65535;
		gains_[0][i] = config.spatialGainCurve.eval(brightness);
	}

	/* Ping-pong between the two gains_ buffers. */
	for (unsigned int i = 0; i < config.diffusion; i++)
		averageGains(gains_[i & 1], gains_[(i & 1) ^ 1], regions_);
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

	/* Update the spatially varying gains. They get written in prepare(). */
	updateGains(stats, config);

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
