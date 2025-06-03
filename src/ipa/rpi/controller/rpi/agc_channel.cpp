/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * AGC/AEC control algorithm
 */

#include "agc_channel.h"

#include <algorithm>
#include <tuple>

#include <libcamera/base/log.h>

#include "libcamera/internal/vector.h"

#include "libipa/colours.h"

#include "../awb_status.h"
#include "../device_status.h"
#include "../histogram.h"
#include "../lux_status.h"
#include "../metadata.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

LOG_DECLARE_CATEGORY(RPiAgc)

int AgcMeteringMode::read(const libcamera::YamlObject &params)
{
	const YamlObject &yamlWeights = params["weights"];

	for (const auto &p : yamlWeights.asList()) {
		auto value = p.get<double>();
		if (!value)
			return -EINVAL;
		weights.push_back(*value);
	}

	return 0;
}

static std::tuple<int, std::string>
readMeteringModes(std::map<std::string, AgcMeteringMode> &metering_modes,
		  const libcamera::YamlObject &params)
{
	std::string first;
	int ret;

	for (const auto &[key, value] : params.asDict()) {
		AgcMeteringMode meteringMode;
		ret = meteringMode.read(value);
		if (ret)
			return { ret, {} };

		metering_modes[key] = std::move(meteringMode);
		if (first.empty())
			first = key;
	}

	return { 0, first };
}

int AgcExposureMode::read(const libcamera::YamlObject &params)
{
	auto value = params["shutter"].getList<double>();
	if (!value)
		return -EINVAL;
	std::transform(value->begin(), value->end(), std::back_inserter(exposureTime),
		       [](double v) { return v * 1us; });

	value = params["gain"].getList<double>();
	if (!value)
		return -EINVAL;
	gain = std::move(*value);

	if (exposureTime.size() < 2 || gain.size() < 2) {
		LOG(RPiAgc, Error)
			<< "AgcExposureMode: must have at least two entries in exposure profile";
		return -EINVAL;
	}

	if (exposureTime.size() != gain.size()) {
		LOG(RPiAgc, Error)
			<< "AgcExposureMode: expect same number of exposure and gain entries in exposure profile";
		return -EINVAL;
	}

	return 0;
}

static std::tuple<int, std::string>
readExposureModes(std::map<std::string, AgcExposureMode> &exposureModes,
		  const libcamera::YamlObject &params)
{
	std::string first;
	int ret;

	for (const auto &[key, value] : params.asDict()) {
		AgcExposureMode exposureMode;
		ret = exposureMode.read(value);
		if (ret)
			return { ret, {} };

		exposureModes[key] = std::move(exposureMode);
		if (first.empty())
			first = key;
	}

	return { 0, first };
}

int AgcConstraint::read(const libcamera::YamlObject &params)
{
	std::string boundString = params["bound"].get<std::string>("");
	transform(boundString.begin(), boundString.end(),
		  boundString.begin(), ::toupper);
	if (boundString != "UPPER" && boundString != "LOWER") {
		LOG(RPiAgc, Error) << "AGC constraint type should be UPPER or LOWER";
		return -EINVAL;
	}
	bound = boundString == "UPPER" ? Bound::UPPER : Bound::LOWER;

	auto value = params["q_lo"].get<double>();
	if (!value)
		return -EINVAL;
	qLo = *value;

	value = params["q_hi"].get<double>();
	if (!value)
		return -EINVAL;
	qHi = *value;

	yTarget = params["y_target"].get<ipa::Pwl>(ipa::Pwl{});
	return yTarget.empty() ? -EINVAL : 0;
}

static std::tuple<int, AgcConstraintMode>
readConstraintMode(const libcamera::YamlObject &params)
{
	AgcConstraintMode mode;
	int ret;

	for (const auto &p : params.asList()) {
		AgcConstraint constraint;
		ret = constraint.read(p);
		if (ret)
			return { ret, {} };

		mode.push_back(std::move(constraint));
	}

	return { 0, mode };
}

static std::tuple<int, std::string>
readConstraintModes(std::map<std::string, AgcConstraintMode> &constraintModes,
		    const libcamera::YamlObject &params)
{
	std::string first;
	int ret;

	for (const auto &[key, value] : params.asDict()) {
		std::tie(ret, constraintModes[key]) = readConstraintMode(value);
		if (ret)
			return { ret, {} };

		if (first.empty())
			first = key;
	}

	return { 0, first };
}

int AgcChannelConstraint::read(const libcamera::YamlObject &params)
{
	auto channelValue = params["channel"].get<unsigned int>();
	if (!channelValue) {
		LOG(RPiAgc, Error) << "AGC channel constraint must have a channel";
		return -EINVAL;
	}
	channel = *channelValue;

	std::string boundString = params["bound"].get<std::string>("");
	transform(boundString.begin(), boundString.end(),
		  boundString.begin(), ::toupper);
	if (boundString != "UPPER" && boundString != "LOWER") {
		LOG(RPiAgc, Error) << "AGC channel constraint type should be UPPER or LOWER";
		return -EINVAL;
	}
	bound = boundString == "UPPER" ? Bound::UPPER : Bound::LOWER;

	auto factorValue = params["factor"].get<double>();
	if (!factorValue) {
		LOG(RPiAgc, Error) << "AGC channel constraint must have a factor";
		return -EINVAL;
	}
	factor = *factorValue;

	return 0;
}

static int readChannelConstraints(std::vector<AgcChannelConstraint> &channelConstraints,
				  const libcamera::YamlObject &params)
{
	for (const auto &p : params.asList()) {
		AgcChannelConstraint constraint;
		int ret = constraint.read(p);
		if (ret)
			return ret;

		channelConstraints.push_back(constraint);
	}

	return 0;
}

int AgcConfig::read(const libcamera::YamlObject &params)
{
	LOG(RPiAgc, Debug) << "AgcConfig";
	int ret;

	std::tie(ret, defaultMeteringMode) =
		readMeteringModes(meteringModes, params["metering_modes"]);
	if (ret)
		return ret;
	std::tie(ret, defaultExposureMode) =
		readExposureModes(exposureModes, params["exposure_modes"]);
	if (ret)
		return ret;
	std::tie(ret, defaultConstraintMode) =
		readConstraintModes(constraintModes, params["constraint_modes"]);
	if (ret)
		return ret;

	if (params.contains("channel_constraints")) {
		ret = readChannelConstraints(channelConstraints, params["channel_constraints"]);
		if (ret)
			return ret;
	}

	yTarget = params["y_target"].get<ipa::Pwl>(ipa::Pwl{});
	if (yTarget.empty())
		return -EINVAL;

	speed = params["speed"].get<double>(0.2);
	startupFrames = params["startup_frames"].get<uint16_t>(10);
	convergenceFrames = params["convergence_frames"].get<unsigned int>(6);
	fastReduceThreshold = params["fast_reduce_threshold"].get<double>(0.4);
	baseEv = params["base_ev"].get<double>(1.0);

	/* Start with quite a low value as ramping up is easier than ramping down. */
	defaultExposureTime = params["default_exposure_time"].get<double>(1000) * 1us;
	defaultAnalogueGain = params["default_analogue_gain"].get<double>(1.0);

	stableRegion = params["stable_region"].get<double>(0.02);

	desaturate = params["desaturate"].get<int>(1);

	return 0;
}

AgcChannel::ExposureValues::ExposureValues()
	: exposureTime(0s), analogueGain(0),
	  totalExposure(0s), totalExposureNoDG(0s)
{
}

AgcChannel::AgcChannel()
	: meteringMode_(nullptr), exposureMode_(nullptr), constraintMode_(nullptr),
	  frameCount_(0), lockCount_(0),
	  lastTargetExposure_(0s), ev_(1.0), flickerPeriod_(0s),
	  maxExposureTime_(0s), fixedExposureTime_(0s), fixedAnalogueGain_(0.0)
{
	/* Set AWB default values in case early frames have no updates in metadata. */
	awb_.gainR = 1.0;
	awb_.gainG = 1.0;
	awb_.gainB = 1.0;

	/*
	 * Setting status_.totalExposureValue_ to zero initially tells us
	 * it's not been calculated yet (i.e. Process hasn't yet run).
	 */
	status_ = {};
	status_.ev = ev_;
}

int AgcChannel::read(const libcamera::YamlObject &params,
		     const Controller::HardwareConfig &hardwareConfig)
{
	int ret = config_.read(params);
	if (ret)
		return ret;

	const Size &size = hardwareConfig.agcZoneWeights;
	for (auto const &modes : config_.meteringModes) {
		if (modes.second.weights.size() != size.width * size.height) {
			LOG(RPiAgc, Error) << "AgcMeteringMode: Incorrect number of weights";
			return -EINVAL;
		}
	}

	/*
	 * Set the config's defaults (which are the first ones it read) as our
	 * current modes, until someone changes them.  (they're all known to
	 * exist at this point)
	 */
	meteringModeName_ = config_.defaultMeteringMode;
	meteringMode_ = &config_.meteringModes[meteringModeName_];
	exposureModeName_ = config_.defaultExposureMode;
	exposureMode_ = &config_.exposureModes[exposureModeName_];
	constraintModeName_ = config_.defaultConstraintMode;
	constraintMode_ = &config_.constraintModes[constraintModeName_];
	/* Set up the "last exposure time/gain" values, in case AGC starts "disabled". */
	status_.exposureTime = config_.defaultExposureTime;
	status_.analogueGain = config_.defaultAnalogueGain;
	return 0;
}

void AgcChannel::disableAutoExposure()
{
	fixedExposureTime_ = status_.exposureTime;
}

void AgcChannel::enableAutoExposure()
{
	fixedExposureTime_ = 0s;
}

bool AgcChannel::autoExposureEnabled() const
{
	return fixedExposureTime_ == 0s;
}

void AgcChannel::disableAutoGain()
{
	fixedAnalogueGain_ = status_.analogueGain;
}

void AgcChannel::enableAutoGain()
{
	fixedAnalogueGain_ = 0;
}

bool AgcChannel::autoGainEnabled() const
{
	return fixedAnalogueGain_ == 0;
}

unsigned int AgcChannel::getConvergenceFrames() const
{
	/*
	 * If exposure time and gain have been explicitly set, there is no
	 * convergence to happen, so no need to drop any frames - return zero.
	 */
	if (fixedExposureTime_ && fixedAnalogueGain_)
		return 0;
	else
		return config_.convergenceFrames;
}

std::vector<double> const &AgcChannel::getWeights() const
{
	/*
	 * In case someone calls setMeteringMode and then this before the
	 * algorithm has run and updated the meteringMode_ pointer.
	 */
	auto it = config_.meteringModes.find(meteringModeName_);
	if (it == config_.meteringModes.end())
		return meteringMode_->weights;
	return it->second.weights;
}

void AgcChannel::setEv(double ev)
{
	ev_ = ev;
}

void AgcChannel::setFlickerPeriod(Duration flickerPeriod)
{
	flickerPeriod_ = flickerPeriod;
}

void AgcChannel::setMaxExposureTime(Duration maxExposureTime)
{
	maxExposureTime_ = maxExposureTime;
}

void AgcChannel::setFixedExposureTime(Duration fixedExposureTime)
{
	fixedExposureTime_ = fixedExposureTime;
	/* Set this in case someone calls disableAuto() straight after. */
	status_.exposureTime = limitExposureTime(fixedExposureTime_);
}

void AgcChannel::setFixedAnalogueGain(double fixedAnalogueGain)
{
	fixedAnalogueGain_ = fixedAnalogueGain;
	/* Set this in case someone calls disableAuto() straight after. */
	status_.analogueGain = limitGain(fixedAnalogueGain);
}

void AgcChannel::setMeteringMode(std::string const &meteringModeName)
{
	meteringModeName_ = meteringModeName;
}

void AgcChannel::setExposureMode(std::string const &exposureModeName)
{
	exposureModeName_ = exposureModeName;
}

void AgcChannel::setConstraintMode(std::string const &constraintModeName)
{
	constraintModeName_ = constraintModeName;
}

void AgcChannel::switchMode(CameraMode const &cameraMode,
			    Metadata *metadata)
{
	/* AGC expects the mode sensitivity always to be non-zero. */
	ASSERT(cameraMode.sensitivity);

	housekeepConfig();

	/*
	 * Store the mode in the local state. We must cache the sensitivity of
	 * of the previous mode for the calculations below.
	 */
	double lastSensitivity = mode_.sensitivity;
	mode_ = cameraMode;

	Duration fixedExposureTime = limitExposureTime(fixedExposureTime_);
	if (fixedExposureTime && fixedAnalogueGain_) {
		/* We're going to reset the algorithm here with these fixed values. */
		fetchAwbStatus(metadata);
		double minColourGain = std::min({ awb_.gainR, awb_.gainG, awb_.gainB, 1.0 });
		ASSERT(minColourGain != 0.0);

		/* This is the equivalent of computeTargetExposure and applyDigitalGain. */
		target_.totalExposureNoDG = fixedExposureTime_ * fixedAnalogueGain_;
		target_.totalExposure = target_.totalExposureNoDG / minColourGain;

		/* Equivalent of filterExposure. This resets any "history". */
		filtered_ = target_;

		/* Equivalent of divideUpExposure. */
		filtered_.exposureTime = fixedExposureTime;
		filtered_.analogueGain = fixedAnalogueGain_;
	} else if (status_.totalExposureValue) {
		/*
		 * On a mode switch, various things could happen:
		 * - the exposure profile might change
		 * - a fixed exposure or gain might be set
		 * - the new mode's sensitivity might be different
		 * We cope with the last of these by scaling the target values. After
		 * that we just need to re-divide the exposure/gain according to the
		 * current exposure profile, which takes care of everything else.
		 */

		double ratio = lastSensitivity / cameraMode.sensitivity;
		target_.totalExposureNoDG *= ratio;
		target_.totalExposure *= ratio;
		filtered_.totalExposureNoDG *= ratio;
		filtered_.totalExposure *= ratio;

		divideUpExposure();
	} else {
		/*
		 * We come through here on startup, when at least one of the
		 * exposure time or gain has not been fixed. We must still
		 * write those values out so that they will be applied
		 * immediately. We supply some arbitrary defaults for any that
		 * weren't set.
		 */

		/* Equivalent of divideUpExposure. */
		filtered_.exposureTime = fixedExposureTime ? fixedExposureTime : config_.defaultExposureTime;
		filtered_.analogueGain = fixedAnalogueGain_ ? fixedAnalogueGain_ : config_.defaultAnalogueGain;
	}

	writeAndFinish(metadata, false);
}

void AgcChannel::prepare(Metadata *imageMetadata)
{
	Duration totalExposureValue = status_.totalExposureValue;
	AgcStatus delayedStatus;
	AgcPrepareStatus prepareStatus;

	/* Fetch the AWB status now because AWB also sets it in the prepare method. */
	fetchAwbStatus(imageMetadata);

	if (!imageMetadata->get("agc.delayed_status", delayedStatus))
		totalExposureValue = delayedStatus.totalExposureValue;

	prepareStatus.digitalGain = 1.0;
	prepareStatus.locked = false;

	if (status_.totalExposureValue) {
		/* Process has run, so we have meaningful values. */
		DeviceStatus deviceStatus;
		if (imageMetadata->get("device.status", deviceStatus) == 0) {
			Duration actualExposure = deviceStatus.exposureTime *
						  deviceStatus.analogueGain;
			if (actualExposure) {
				double digitalGain = totalExposureValue / actualExposure;
				LOG(RPiAgc, Debug) << "Want total exposure " << totalExposureValue;
				/*
				 * Never ask for a gain < 1.0, and also impose
				 * some upper limit. Make it customisable?
				 */
				prepareStatus.digitalGain = std::max(1.0, std::min(digitalGain, 4.0));
				LOG(RPiAgc, Debug) << "Actual exposure " << actualExposure;
				LOG(RPiAgc, Debug) << "Use digitalGain " << prepareStatus.digitalGain;
				LOG(RPiAgc, Debug) << "Effective exposure "
						   << actualExposure * prepareStatus.digitalGain;
				/* Decide whether AEC/AGC has converged. */
				prepareStatus.locked = updateLockStatus(deviceStatus);
			}
		} else
			LOG(RPiAgc, Warning) << "AgcChannel: no device metadata";
		imageMetadata->set("agc.prepare_status", prepareStatus);
	}
}

void AgcChannel::process(StatisticsPtr &stats, DeviceStatus const &deviceStatus,
			 Metadata *imageMetadata,
			 const AgcChannelTotalExposures &channelTotalExposures)
{
	frameCount_++;
	/*
	 * First a little bit of housekeeping, fetching up-to-date settings and
	 * configuration, that kind of thing.
	 */
	housekeepConfig();
	/* Get the current exposure values for the frame that's just arrived. */
	fetchCurrentExposure(deviceStatus);
	/* Compute the total gain we require relative to the current exposure. */
	double gain, targetY;
	computeGain(stats, imageMetadata, gain, targetY);
	/* Now compute the target (final) exposure which we think we want. */
	computeTargetExposure(gain);
	/* The results have to be filtered so as not to change too rapidly. */
	filterExposure();
	/*
	 * We may be asked to limit the exposure using other channels. If another channel
	 * determines our upper bound we may want to know this later.
	 */
	bool channelBound = applyChannelConstraints(channelTotalExposures);
	/*
	 * Some of the exposure has to be applied as digital gain, so work out
	 * what that is. It also tells us whether it's trying to desaturate the image
	 * more quickly, which can only happen when another channel is not limiting us.
	 */
	bool desaturate = applyDigitalGain(gain, targetY, channelBound);
	/*
	 * The last thing is to divide up the exposure value into a exposure time
	 * and analogue gain, according to the current exposure mode.
	 */
	divideUpExposure();
	/* Finally advertise what we've done. */
	writeAndFinish(imageMetadata, desaturate);
}

bool AgcChannel::updateLockStatus(DeviceStatus const &deviceStatus)
{
	const double errorFactor = 0.10; /* make these customisable? */
	const int maxLockCount = 5;
	/* Reset "lock count" when we exceed this multiple of errorFactor */
	const double resetMargin = 1.5;

	/* Add 200us to the exposure time error to allow for line quantisation. */
	Duration exposureError = lastDeviceStatus_.exposureTime * errorFactor + 200us;
	double gainError = lastDeviceStatus_.analogueGain * errorFactor;
	Duration targetError = lastTargetExposure_ * errorFactor;

	/*
	 * Note that we don't know the exposure/gain limits of the sensor, so
	 * the values we keep requesting may be unachievable. For this reason
	 * we only insist that we're close to values in the past few frames.
	 */
	if (deviceStatus.exposureTime > lastDeviceStatus_.exposureTime - exposureError &&
	    deviceStatus.exposureTime < lastDeviceStatus_.exposureTime + exposureError &&
	    deviceStatus.analogueGain > lastDeviceStatus_.analogueGain - gainError &&
	    deviceStatus.analogueGain < lastDeviceStatus_.analogueGain + gainError &&
	    status_.targetExposureValue > lastTargetExposure_ - targetError &&
	    status_.targetExposureValue < lastTargetExposure_ + targetError)
		lockCount_ = std::min(lockCount_ + 1, maxLockCount);
	else if (deviceStatus.exposureTime < lastDeviceStatus_.exposureTime - resetMargin * exposureError ||
		 deviceStatus.exposureTime > lastDeviceStatus_.exposureTime + resetMargin * exposureError ||
		 deviceStatus.analogueGain < lastDeviceStatus_.analogueGain - resetMargin * gainError ||
		 deviceStatus.analogueGain > lastDeviceStatus_.analogueGain + resetMargin * gainError ||
		 status_.targetExposureValue < lastTargetExposure_ - resetMargin * targetError ||
		 status_.targetExposureValue > lastTargetExposure_ + resetMargin * targetError)
		lockCount_ = 0;

	lastDeviceStatus_ = deviceStatus;
	lastTargetExposure_ = status_.targetExposureValue;

	LOG(RPiAgc, Debug) << "Lock count updated to " << lockCount_;
	return lockCount_ == maxLockCount;
}

void AgcChannel::housekeepConfig()
{
	/* First fetch all the up-to-date settings, so no one else has to do it. */
	status_.ev = ev_;
	status_.fixedExposureTime = limitExposureTime(fixedExposureTime_);
	status_.fixedAnalogueGain = fixedAnalogueGain_;
	status_.flickerPeriod = flickerPeriod_;
	LOG(RPiAgc, Debug) << "ev " << status_.ev << " fixedExposureTime "
			   << status_.fixedExposureTime << " fixedAnalogueGain "
			   << status_.fixedAnalogueGain;
	/*
	 * Make sure the "mode" pointers point to the up-to-date things, if
	 * they've changed.
	 */
	if (meteringModeName_ != status_.meteringMode) {
		auto it = config_.meteringModes.find(meteringModeName_);
		if (it == config_.meteringModes.end()) {
			LOG(RPiAgc, Warning) << "No metering mode " << meteringModeName_;
			meteringModeName_ = status_.meteringMode;
		} else {
			meteringMode_ = &it->second;
			status_.meteringMode = meteringModeName_;
		}
	}
	if (exposureModeName_ != status_.exposureMode) {
		auto it = config_.exposureModes.find(exposureModeName_);
		if (it == config_.exposureModes.end()) {
			LOG(RPiAgc, Warning) << "No exposure profile " << exposureModeName_;
			exposureModeName_ = status_.exposureMode;
		} else {
			exposureMode_ = &it->second;
			status_.exposureMode = exposureModeName_;
		}
	}
	if (constraintModeName_ != status_.constraintMode) {
		auto it = config_.constraintModes.find(constraintModeName_);
		if (it == config_.constraintModes.end()) {
			LOG(RPiAgc, Warning) << "No constraint list " << constraintModeName_;
			constraintModeName_ = status_.constraintMode;
		} else {
			constraintMode_ = &it->second;
			status_.constraintMode = constraintModeName_;
		}
	}
	LOG(RPiAgc, Debug) << "exposureMode "
			   << exposureModeName_ << " constraintMode "
			   << constraintModeName_ << " meteringMode "
			   << meteringModeName_;
}

void AgcChannel::fetchCurrentExposure(DeviceStatus const &deviceStatus)
{
	current_.exposureTime = deviceStatus.exposureTime;
	current_.analogueGain = deviceStatus.analogueGain;
	current_.totalExposure = 0s; /* this value is unused */
	current_.totalExposureNoDG = current_.exposureTime * current_.analogueGain;
}

void AgcChannel::fetchAwbStatus(Metadata *imageMetadata)
{
	if (imageMetadata->get("awb.status", awb_) != 0)
		LOG(RPiAgc, Debug) << "No AWB status found";
}

static double computeInitialY(StatisticsPtr &stats, AwbStatus const &awb,
			      std::vector<double> &weights, double gain)
{
	constexpr uint64_t maxVal = 1 << Statistics::NormalisationFactorPow2;

	/*
	 * If we have no AGC region stats, but do have a a Y histogram, use that
	 * directly to caluclate the mean Y value of the image.
	 */
	if (!stats->agcRegions.numRegions() && stats->yHist.bins()) {
		/*
		 * When the gain is applied to the histogram, anything below minBin
		 * will scale up directly with the gain, but anything above that
		 * will saturate into the top bin.
		 */
		auto &hist = stats->yHist;
		double minBin = std::min(1.0, 1.0 / gain) * hist.bins();
		double binMean = hist.interBinMean(0.0, minBin);
		double numUnsaturated = hist.cumulativeFreq(minBin);
		/* This term is from all the pixels that won't saturate. */
		double ySum = binMean * gain * numUnsaturated;
		/* And add the ones that will saturate. */
		ySum += (hist.total() - numUnsaturated) * hist.bins();
		return ySum / hist.total() / hist.bins();
	}

	ASSERT(weights.size() == stats->agcRegions.numRegions());

	/*
	 * Note that the weights are applied by the IPA to the statistics directly,
	 * before they are given to us here.
	 */
	RGB<double> sum{ 0.0 };
	double pixelSum = 0;
	for (unsigned int i = 0; i < stats->agcRegions.numRegions(); i++) {
		auto &region = stats->agcRegions.get(i);
		sum.r() += std::min<double>(region.val.rSum * gain, (maxVal - 1) * region.counted);
		sum.g() += std::min<double>(region.val.gSum * gain, (maxVal - 1) * region.counted);
		sum.b() += std::min<double>(region.val.bSum * gain, (maxVal - 1) * region.counted);
		pixelSum += region.counted;
	}
	if (pixelSum == 0.0) {
		LOG(RPiAgc, Warning) << "computeInitialY: pixelSum is zero";
		return 0;
	}

	/* Factor in the AWB correction if needed. */
	if (stats->agcStatsPos == Statistics::AgcStatsPos::PreWb)
		sum *= RGB<double>{ { awb.gainR, awb.gainG, awb.gainB } };

	double ySum = ipa::rec601LuminanceFromRGB(sum);

	return ySum / pixelSum / (1 << 16);
}

/*
 * We handle extra gain through EV by adjusting our Y targets. However, you
 * simply can't monitor histograms once they get very close to (or beyond!)
 * saturation, so we clamp the Y targets to this value. It does mean that EV
 * increases don't necessarily do quite what you might expect in certain
 * (contrived) cases.
 */

static constexpr double EvGainYTargetLimit = 0.9;

static double constraintComputeGain(AgcConstraint &c, const Histogram &h, double lux,
				    double evGain, double &targetY)
{
	targetY = c.yTarget.eval(c.yTarget.domain().clamp(lux));
	targetY = std::min(EvGainYTargetLimit, targetY * evGain);
	double iqm = h.interQuantileMean(c.qLo, c.qHi);
	return (targetY * h.bins()) / iqm;
}

void AgcChannel::computeGain(StatisticsPtr &statistics, Metadata *imageMetadata,
			     double &gain, double &targetY)
{
	struct LuxStatus lux = {};
	lux.lux = 400; /* default lux level to 400 in case no metadata found */
	if (imageMetadata->get("lux.status", lux) != 0)
		LOG(RPiAgc, Warning) << "No lux level found";
	const Histogram &h = statistics->yHist;
	double evGain = status_.ev * config_.baseEv;
	/*
	 * The initial gain and target_Y come from some of the regions. After
	 * that we consider the histogram constraints.
	 */
	targetY = config_.yTarget.eval(config_.yTarget.domain().clamp(lux.lux));
	targetY = std::min(EvGainYTargetLimit, targetY * evGain);

	/*
	 * Do this calculation a few times as brightness increase can be
	 * non-linear when there are saturated regions.
	 */
	gain = 1.0;
	for (int i = 0; i < 8; i++) {
		double initialY = computeInitialY(statistics, awb_, meteringMode_->weights, gain);
		double extraGain = std::min(10.0, targetY / (initialY + .001));
		gain *= extraGain;
		LOG(RPiAgc, Debug) << "Initial Y " << initialY << " target " << targetY
				   << " gives gain " << gain;
		if (extraGain < 1.01) /* close enough */
			break;
	}

	for (auto &c : *constraintMode_) {
		double newTargetY;
		double newGain = constraintComputeGain(c, h, lux.lux, evGain, newTargetY);
		LOG(RPiAgc, Debug) << "Constraint has target_Y "
				   << newTargetY << " giving gain " << newGain;
		if (c.bound == AgcConstraint::Bound::LOWER && newGain > gain) {
			LOG(RPiAgc, Debug) << "Lower bound constraint adopted";
			gain = newGain;
			targetY = newTargetY;
		} else if (c.bound == AgcConstraint::Bound::UPPER && newGain < gain) {
			LOG(RPiAgc, Debug) << "Upper bound constraint adopted";
			gain = newGain;
			targetY = newTargetY;
		}
	}
	LOG(RPiAgc, Debug) << "Final gain " << gain << " (target_Y " << targetY << " ev "
			   << status_.ev << " base_ev " << config_.baseEv
			   << ")";
}

void AgcChannel::computeTargetExposure(double gain)
{
	if (status_.fixedExposureTime && status_.fixedAnalogueGain) {
		/*
		 * When analogue gain and exposure time are both fixed, we need
		 * to drive the total exposure so that we end up with a digital
		 * gain of at least 1/minColourGain. Otherwise we'd desaturate
		 * channels causing white to go cyan or magenta.
		 */
		double minColourGain = std::min({ awb_.gainR, awb_.gainG, awb_.gainB, 1.0 });
		ASSERT(minColourGain != 0.0);
		target_.totalExposure =
			status_.fixedExposureTime * status_.fixedAnalogueGain / minColourGain;
	} else {
		/*
		 * The statistics reflect the image without digital gain, so the final
		 * total exposure we're aiming for is:
		 */
		target_.totalExposure = current_.totalExposureNoDG * gain;
		/* The final target exposure is also limited to what the exposure mode allows. */
		Duration maxExposureTime = status_.fixedExposureTime
					 ? status_.fixedExposureTime
					 : exposureMode_->exposureTime.back();
		maxExposureTime = limitExposureTime(maxExposureTime);
		Duration maxTotalExposure =
			maxExposureTime *
			(status_.fixedAnalogueGain != 0.0
				 ? status_.fixedAnalogueGain
				 : exposureMode_->gain.back());
		target_.totalExposure = std::min(target_.totalExposure, maxTotalExposure);
	}
	LOG(RPiAgc, Debug) << "Target totalExposure " << target_.totalExposure;
}

bool AgcChannel::applyChannelConstraints(const AgcChannelTotalExposures &channelTotalExposures)
{
	bool channelBound = false;
	LOG(RPiAgc, Debug)
		<< "Total exposure before channel constraints " << filtered_.totalExposure;

	for (const auto &constraint : config_.channelConstraints) {
		LOG(RPiAgc, Debug)
			<< "Check constraint: channel " << constraint.channel << " bound "
			<< (constraint.bound == AgcChannelConstraint::Bound::UPPER ? "UPPER" : "LOWER")
			<< " factor " << constraint.factor;
		if (constraint.channel >= channelTotalExposures.size() ||
		    !channelTotalExposures[constraint.channel]) {
			LOG(RPiAgc, Debug) << "no such channel or no exposure available- skipped";
			continue;
		}

		libcamera::utils::Duration limitExposure =
			channelTotalExposures[constraint.channel] * constraint.factor;
		LOG(RPiAgc, Debug) << "Limit exposure " << limitExposure;
		if ((constraint.bound == AgcChannelConstraint::Bound::UPPER &&
		     filtered_.totalExposure > limitExposure) ||
		    (constraint.bound == AgcChannelConstraint::Bound::LOWER &&
		     filtered_.totalExposure < limitExposure)) {
			filtered_.totalExposure = limitExposure;
			LOG(RPiAgc, Debug) << "Constraint applies";
			channelBound = true;
		} else
			LOG(RPiAgc, Debug) << "Constraint does not apply";
	}

	LOG(RPiAgc, Debug)
		<< "Total exposure after channel constraints " << filtered_.totalExposure;

	return channelBound;
}

bool AgcChannel::applyDigitalGain(double gain, double targetY, bool channelBound)
{
	double minColourGain = std::min({ awb_.gainR, awb_.gainG, awb_.gainB, 1.0 });
	ASSERT(minColourGain != 0.0);
	double dg = 1.0 / minColourGain;
	/*
	 * I think this pipeline subtracts black level and rescales before we
	 * get the stats, so no need to worry about it.
	 */
	LOG(RPiAgc, Debug) << "after AWB, target dg " << dg << " gain " << gain
			   << " target_Y " << targetY;
	/*
	 * Finally, if we're trying to reduce exposure but the target_Y is
	 * "close" to 1.0, then the gain computed for that constraint will be
	 * only slightly less than one, because the measured Y can never be
	 * larger than 1.0. When this happens, demand a large digital gain so
	 * that the exposure can be reduced, de-saturating the image much more
	 * quickly (and we then approach the correct value more quickly from
	 * below).
	 */
	bool desaturate = false;
	if (config_.desaturate)
		desaturate = !channelBound &&
			     targetY > config_.fastReduceThreshold && gain < sqrt(targetY);
	if (desaturate)
		dg /= config_.fastReduceThreshold;
	LOG(RPiAgc, Debug) << "Digital gain " << dg << " desaturate? " << desaturate;
	filtered_.totalExposureNoDG = filtered_.totalExposure / dg;
	LOG(RPiAgc, Debug) << "Target totalExposureNoDG " << filtered_.totalExposureNoDG;
	return desaturate;
}

void AgcChannel::filterExposure()
{
	double speed = config_.speed;
	double stableRegion = config_.stableRegion;

	/*
	 * AGC adapts instantly if both exposure time and gain are directly
	 * specified or we're in the startup phase. Also disable the stable
	 * region, because we want to reflect any user exposure/gain updates,
	 * however small.
	 */
	if ((status_.fixedExposureTime && status_.fixedAnalogueGain) ||
	    frameCount_ <= config_.startupFrames) {
		speed = 1.0;
		stableRegion = 0.0;
	}
	if (!filtered_.totalExposure) {
		filtered_.totalExposure = target_.totalExposure;
	} else if (filtered_.totalExposure * (1.0 - stableRegion) < target_.totalExposure &&
		   filtered_.totalExposure * (1.0 + stableRegion) > target_.totalExposure) {
		/* Total exposure must change by more than this or we leave it alone. */
	} else {
		/*
		 * If close to the result go faster, to save making so many
		 * micro-adjustments on the way. (Make this customisable?)
		 */
		if (filtered_.totalExposure < 1.2 * target_.totalExposure &&
		    filtered_.totalExposure > 0.8 * target_.totalExposure)
			speed = sqrt(speed);
		filtered_.totalExposure = speed * target_.totalExposure +
					  filtered_.totalExposure * (1.0 - speed);
	}
	LOG(RPiAgc, Debug) << "After filtering, totalExposure " << filtered_.totalExposure
			   << " no dg " << filtered_.totalExposureNoDG;
}

void AgcChannel::divideUpExposure()
{
	/*
	 * Sending the fixed exposure time/gain cases through the same code may
	 * seem unnecessary, but it will make more sense when extend this to
	 * cover variable aperture.
	 */
	Duration exposureValue = filtered_.totalExposureNoDG;
	Duration exposureTime;
	double analogueGain;
	exposureTime = status_.fixedExposureTime ? status_.fixedExposureTime
						 : exposureMode_->exposureTime[0];
	exposureTime = limitExposureTime(exposureTime);
	analogueGain = status_.fixedAnalogueGain != 0.0 ? status_.fixedAnalogueGain
							: exposureMode_->gain[0];
	analogueGain = limitGain(analogueGain);
	if (exposureTime * analogueGain < exposureValue) {
		for (unsigned int stage = 1;
		     stage < exposureMode_->gain.size(); stage++) {
			if (!status_.fixedExposureTime) {
				Duration stageExposureTime =
					limitExposureTime(exposureMode_->exposureTime[stage]);
				if (stageExposureTime * analogueGain >= exposureValue) {
					exposureTime = exposureValue / analogueGain;
					break;
				}
				exposureTime = stageExposureTime;
			}
			if (status_.fixedAnalogueGain == 0.0) {
				if (exposureMode_->gain[stage] * exposureTime >= exposureValue) {
					analogueGain = exposureValue / exposureTime;
					break;
				}
				analogueGain = exposureMode_->gain[stage];
				analogueGain = limitGain(analogueGain);
			}
		}
	}
	LOG(RPiAgc, Debug)
		<< "Divided up exposure time and gain are " << exposureTime
		<< " and " << analogueGain;
	/*
	 * Finally adjust exposure time for flicker avoidance (require both
	 * exposure time and gain not to be fixed).
	 */
	if (!status_.fixedExposureTime && !status_.fixedAnalogueGain &&
	    status_.flickerPeriod) {
		int flickerPeriods = exposureTime / status_.flickerPeriod;
		if (flickerPeriods) {
			Duration newExposureTime = flickerPeriods * status_.flickerPeriod;
			analogueGain *= exposureTime / newExposureTime;
			/*
			 * We should still not allow the ag to go over the
			 * largest value in the exposure mode. Note that this
			 * may force more of the total exposure into the digital
			 * gain as a side-effect.
			 */
			analogueGain = std::min(analogueGain, exposureMode_->gain.back());
			analogueGain = limitGain(analogueGain);
			exposureTime = newExposureTime;
		}
		LOG(RPiAgc, Debug) << "After flicker avoidance, exposure time "
				   << exposureTime << " gain " << analogueGain;
	}
	filtered_.exposureTime = exposureTime;
	filtered_.analogueGain = analogueGain;
}

void AgcChannel::writeAndFinish(Metadata *imageMetadata, bool desaturate)
{
	status_.totalExposureValue = filtered_.totalExposure;
	status_.targetExposureValue = desaturate ? 0s : target_.totalExposure;
	status_.exposureTime = filtered_.exposureTime;
	status_.analogueGain = filtered_.analogueGain;
	/*
	 * Write to metadata as well, in case anyone wants to update the camera
	 * immediately.
	 */
	imageMetadata->set("agc.status", status_);
	LOG(RPiAgc, Debug) << "Output written, total exposure requested is "
			   << filtered_.totalExposure;
	LOG(RPiAgc, Debug) << "Camera exposure update: exposure time " << filtered_.exposureTime
			   << " analogue gain " << filtered_.analogueGain;
}

Duration AgcChannel::limitExposureTime(Duration exposureTime)
{
	/*
	 * exposureTime == 0 is a special case for fixed exposure time values,
	 * and must pass through unchanged.
	 */
	if (!exposureTime)
		return exposureTime;

	exposureTime = std::clamp(exposureTime, mode_.minExposureTime, maxExposureTime_);
	return exposureTime;
}

double AgcChannel::limitGain(double gain) const
{
	/*
	 * Only limit the lower bounds of the gain value to what the sensor
	 * limits. The upper bound on analogue gain will be made up with
	 * additional digital gain applied by the ISP.
	 *
	 * gain == 0.0 is a special case for fixed exposure time values, and
	 * must pass through unchanged.
	 */
	if (!gain)
		return gain;

	gain = std::max(gain, mode_.minAnalogueGain);
	return gain;
}
