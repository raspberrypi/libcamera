/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * agc.cpp - AGC/AEC control algorithm
 */

#include <algorithm>
#include <map>
#include <tuple>

#include <linux/bcm2835-isp.h>

#include <libcamera/base/log.h>

#include "../awb_status.h"
#include "../device_status.h"
#include "../histogram.h"
#include "../lux_status.h"
#include "../metadata.h"

#include "agc.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(RPiAgc)

#define NAME "rpi.agc"

static constexpr unsigned int PipelineBits = 13; /* seems to be a 13-bit pipeline */

int AgcMeteringMode::read(const libcamera::YamlObject &params)
{
	const YamlObject &yamlWeights = params["weights"];
	if (yamlWeights.size() != AgcStatsSize) {
		LOG(RPiAgc, Error) << "AgcMeteringMode: Incorrect number of weights";
		return -EINVAL;
	}

	unsigned int num = 0;
	for (const auto &p : yamlWeights.asList()) {
		auto value = p.get<double>();
		if (!value)
			return -EINVAL;
		weights[num++] = *value;
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
	std::transform(value->begin(), value->end(), std::back_inserter(shutter),
		       [](double v) { return v * 1us; });

	value = params["gain"].getList<double>();
	if (!value)
		return -EINVAL;
	gain = std::move(*value);

	if (shutter.size() < 2 || gain.size() < 2) {
		LOG(RPiAgc, Error)
			<< "AgcExposureMode: must have at least two entries in exposure profile";
		return -EINVAL;
	}

	if (shutter.size() != gain.size()) {
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

	return yTarget.read(params["y_target"]);
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

	ret = yTarget.read(params["y_target"]);
	if (ret)
		return ret;

	speed = params["speed"].get<double>(0.2);
	startupFrames = params["startup_frames"].get<uint16_t>(10);
	convergenceFrames = params["convergence_frames"].get<unsigned int>(6);
	fastReduceThreshold = params["fast_reduce_threshold"].get<double>(0.4);
	baseEv = params["base_ev"].get<double>(1.0);

	/* Start with quite a low value as ramping up is easier than ramping down. */
	defaultExposureTime = params["default_exposure_time"].get<double>(1000) * 1us;
	defaultAnalogueGain = params["default_analogue_gain"].get<double>(1.0);

	return 0;
}

Agc::ExposureValues::ExposureValues()
	: shutter(0s), analogueGain(0),
	  totalExposure(0s), totalExposureNoDG(0s)
{
}

Agc::Agc(Controller *controller)
	: AgcAlgorithm(controller), meteringMode_(nullptr),
	  exposureMode_(nullptr), constraintMode_(nullptr),
	  frameCount_(0), lockCount_(0),
	  lastTargetExposure_(0s), lastSensitivity_(0.0),
	  ev_(1.0), flickerPeriod_(0s),
	  maxShutter_(0s), fixedShutter_(0s), fixedAnalogueGain_(0.0)
{
	memset(&awb_, 0, sizeof(awb_));
	/*
	 * Setting status_.totalExposureValue_ to zero initially tells us
	 * it's not been calculated yet (i.e. Process hasn't yet run).
	 */
	memset(&status_, 0, sizeof(status_));
	status_.ev = ev_;
}

char const *Agc::name() const
{
	return NAME;
}

int Agc::read(const libcamera::YamlObject &params)
{
	LOG(RPiAgc, Debug) << "Agc";

	int ret = config_.read(params);
	if (ret)
		return ret;

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
	/* Set up the "last shutter/gain" values, in case AGC starts "disabled". */
	status_.shutterTime = config_.defaultExposureTime;
	status_.analogueGain = config_.defaultAnalogueGain;
	return 0;
}

void Agc::disableAuto()
{
	fixedShutter_ = status_.shutterTime;
	fixedAnalogueGain_ = status_.analogueGain;
}

void Agc::enableAuto()
{
	fixedShutter_ = 0s;
	fixedAnalogueGain_ = 0;
}

unsigned int Agc::getConvergenceFrames() const
{
	/*
	 * If shutter and gain have been explicitly set, there is no
	 * convergence to happen, so no need to drop any frames - return zero.
	 */
	if (fixedShutter_ && fixedAnalogueGain_)
		return 0;
	else
		return config_.convergenceFrames;
}

void Agc::setEv(double ev)
{
	ev_ = ev;
}

void Agc::setFlickerPeriod(Duration flickerPeriod)
{
	flickerPeriod_ = flickerPeriod;
}

void Agc::setMaxShutter(Duration maxShutter)
{
	maxShutter_ = maxShutter;
}

void Agc::setFixedShutter(Duration fixedShutter)
{
	fixedShutter_ = fixedShutter;
	/* Set this in case someone calls disableAuto() straight after. */
	status_.shutterTime = clipShutter(fixedShutter_);
}

void Agc::setFixedAnalogueGain(double fixedAnalogueGain)
{
	fixedAnalogueGain_ = fixedAnalogueGain;
	/* Set this in case someone calls disableAuto() straight after. */
	status_.analogueGain = fixedAnalogueGain;
}

void Agc::setMeteringMode(std::string const &meteringModeName)
{
	meteringModeName_ = meteringModeName;
}

void Agc::setExposureMode(std::string const &exposureModeName)
{
	exposureModeName_ = exposureModeName;
}

void Agc::setConstraintMode(std::string const &constraintModeName)
{
	constraintModeName_ = constraintModeName;
}

void Agc::switchMode(CameraMode const &cameraMode,
		     Metadata *metadata)
{
	/* AGC expects the mode sensitivity always to be non-zero. */
	ASSERT(cameraMode.sensitivity);

	housekeepConfig();

	Duration fixedShutter = clipShutter(fixedShutter_);
	if (fixedShutter && fixedAnalogueGain_) {
		/* We're going to reset the algorithm here with these fixed values. */

		fetchAwbStatus(metadata);
		double minColourGain = std::min({ awb_.gainR, awb_.gainG, awb_.gainB, 1.0 });
		ASSERT(minColourGain != 0.0);

		/* This is the equivalent of computeTargetExposure and applyDigitalGain. */
		target_.totalExposureNoDG = fixedShutter_ * fixedAnalogueGain_;
		target_.totalExposure = target_.totalExposureNoDG / minColourGain;

		/* Equivalent of filterExposure. This resets any "history". */
		filtered_ = target_;

		/* Equivalent of divideUpExposure. */
		filtered_.shutter = fixedShutter;
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

		double ratio = lastSensitivity_ / cameraMode.sensitivity;
		target_.totalExposureNoDG *= ratio;
		target_.totalExposure *= ratio;
		filtered_.totalExposureNoDG *= ratio;
		filtered_.totalExposure *= ratio;

		divideUpExposure();
	} else {
		/*
		 * We come through here on startup, when at least one of the shutter
		 * or gain has not been fixed. We must still write those values out so
		 * that they will be applied immediately. We supply some arbitrary defaults
		 * for any that weren't set.
		 */

		/* Equivalent of divideUpExposure. */
		filtered_.shutter = fixedShutter ? fixedShutter : config_.defaultExposureTime;
		filtered_.analogueGain = fixedAnalogueGain_ ? fixedAnalogueGain_ : config_.defaultAnalogueGain;
	}

	writeAndFinish(metadata, false);

	/* We must remember the sensitivity of this mode for the next SwitchMode. */
	lastSensitivity_ = cameraMode.sensitivity;
}

void Agc::prepare(Metadata *imageMetadata)
{
	Duration totalExposureValue = status_.totalExposureValue;
	AgcStatus delayedStatus;

	if (!imageMetadata->get("agc.delayed_status", delayedStatus))
		totalExposureValue = delayedStatus.totalExposureValue;

	status_.digitalGain = 1.0;
	fetchAwbStatus(imageMetadata); /* always fetch it so that Process knows it's been done */

	if (status_.totalExposureValue) {
		/* Process has run, so we have meaningful values. */
		DeviceStatus deviceStatus;
		if (imageMetadata->get("device.status", deviceStatus) == 0) {
			Duration actualExposure = deviceStatus.shutterSpeed *
						  deviceStatus.analogueGain;
			if (actualExposure) {
				status_.digitalGain = totalExposureValue / actualExposure;
				LOG(RPiAgc, Debug) << "Want total exposure " << totalExposureValue;
				/*
				 * Never ask for a gain < 1.0, and also impose
				 * some upper limit. Make it customisable?
				 */
				status_.digitalGain = std::max(1.0, std::min(status_.digitalGain, 4.0));
				LOG(RPiAgc, Debug) << "Actual exposure " << actualExposure;
				LOG(RPiAgc, Debug) << "Use digitalGain " << status_.digitalGain;
				LOG(RPiAgc, Debug) << "Effective exposure "
						   << actualExposure * status_.digitalGain;
				/* Decide whether AEC/AGC has converged. */
				updateLockStatus(deviceStatus);
			}
		} else
			LOG(RPiAgc, Warning) << name() << ": no device metadata";
		imageMetadata->set("agc.status", status_);
	}
}

void Agc::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	frameCount_++;
	/*
	 * First a little bit of housekeeping, fetching up-to-date settings and
	 * configuration, that kind of thing.
	 */
	housekeepConfig();
	/* Get the current exposure values for the frame that's just arrived. */
	fetchCurrentExposure(imageMetadata);
	/* Compute the total gain we require relative to the current exposure. */
	double gain, targetY;
	computeGain(stats.get(), imageMetadata, gain, targetY);
	/* Now compute the target (final) exposure which we think we want. */
	computeTargetExposure(gain);
	/*
	 * Some of the exposure has to be applied as digital gain, so work out
	 * what that is. This function also tells us whether it's decided to
	 * "desaturate" the image more quickly.
	 */
	bool desaturate = applyDigitalGain(gain, targetY);
	/* The results have to be filtered so as not to change too rapidly. */
	filterExposure(desaturate);
	/*
	 * The last thing is to divide up the exposure value into a shutter time
	 * and analogue gain, according to the current exposure mode.
	 */
	divideUpExposure();
	/* Finally advertise what we've done. */
	writeAndFinish(imageMetadata, desaturate);
}

void Agc::updateLockStatus(DeviceStatus const &deviceStatus)
{
	const double errorFactor = 0.10; /* make these customisable? */
	const int maxLockCount = 5;
	/* Reset "lock count" when we exceed this multiple of errorFactor */
	const double resetMargin = 1.5;

	/* Add 200us to the exposure time error to allow for line quantisation. */
	Duration exposureError = lastDeviceStatus_.shutterSpeed * errorFactor + 200us;
	double gainError = lastDeviceStatus_.analogueGain * errorFactor;
	Duration targetError = lastTargetExposure_ * errorFactor;

	/*
	 * Note that we don't know the exposure/gain limits of the sensor, so
	 * the values we keep requesting may be unachievable. For this reason
	 * we only insist that we're close to values in the past few frames.
	 */
	if (deviceStatus.shutterSpeed > lastDeviceStatus_.shutterSpeed - exposureError &&
	    deviceStatus.shutterSpeed < lastDeviceStatus_.shutterSpeed + exposureError &&
	    deviceStatus.analogueGain > lastDeviceStatus_.analogueGain - gainError &&
	    deviceStatus.analogueGain < lastDeviceStatus_.analogueGain + gainError &&
	    status_.targetExposureValue > lastTargetExposure_ - targetError &&
	    status_.targetExposureValue < lastTargetExposure_ + targetError)
		lockCount_ = std::min(lockCount_ + 1, maxLockCount);
	else if (deviceStatus.shutterSpeed < lastDeviceStatus_.shutterSpeed - resetMargin * exposureError ||
		 deviceStatus.shutterSpeed > lastDeviceStatus_.shutterSpeed + resetMargin * exposureError ||
		 deviceStatus.analogueGain < lastDeviceStatus_.analogueGain - resetMargin * gainError ||
		 deviceStatus.analogueGain > lastDeviceStatus_.analogueGain + resetMargin * gainError ||
		 status_.targetExposureValue < lastTargetExposure_ - resetMargin * targetError ||
		 status_.targetExposureValue > lastTargetExposure_ + resetMargin * targetError)
		lockCount_ = 0;

	lastDeviceStatus_ = deviceStatus;
	lastTargetExposure_ = status_.targetExposureValue;

	LOG(RPiAgc, Debug) << "Lock count updated to " << lockCount_;
	status_.locked = lockCount_ == maxLockCount;
}

static void copyString(std::string const &s, char *d, size_t size)
{
	size_t length = s.copy(d, size - 1);
	d[length] = '\0';
}

void Agc::housekeepConfig()
{
	/* First fetch all the up-to-date settings, so no one else has to do it. */
	status_.ev = ev_;
	status_.fixedShutter = clipShutter(fixedShutter_);
	status_.fixedAnalogueGain = fixedAnalogueGain_;
	status_.flickerPeriod = flickerPeriod_;
	LOG(RPiAgc, Debug) << "ev " << status_.ev << " fixedShutter "
			   << status_.fixedShutter << " fixedAnalogueGain "
			   << status_.fixedAnalogueGain;
	/*
	 * Make sure the "mode" pointers point to the up-to-date things, if
	 * they've changed.
	 */
	if (strcmp(meteringModeName_.c_str(), status_.meteringMode)) {
		auto it = config_.meteringModes.find(meteringModeName_);
		if (it == config_.meteringModes.end())
			LOG(RPiAgc, Fatal) << "No metering mode " << meteringModeName_;
		meteringMode_ = &it->second;
		copyString(meteringModeName_, status_.meteringMode,
			   sizeof(status_.meteringMode));
	}
	if (strcmp(exposureModeName_.c_str(), status_.exposureMode)) {
		auto it = config_.exposureModes.find(exposureModeName_);
		if (it == config_.exposureModes.end())
			LOG(RPiAgc, Fatal) << "No exposure profile " << exposureModeName_;
		exposureMode_ = &it->second;
		copyString(exposureModeName_, status_.exposureMode,
			   sizeof(status_.exposureMode));
	}
	if (strcmp(constraintModeName_.c_str(), status_.constraintMode)) {
		auto it =
			config_.constraintModes.find(constraintModeName_);
		if (it == config_.constraintModes.end())
			LOG(RPiAgc, Fatal) << "No constraint list " << constraintModeName_;
		constraintMode_ = &it->second;
		copyString(constraintModeName_, status_.constraintMode,
			   sizeof(status_.constraintMode));
	}
	LOG(RPiAgc, Debug) << "exposureMode "
			   << exposureModeName_ << " constraintMode "
			   << constraintModeName_ << " meteringMode "
			   << meteringModeName_;
}

void Agc::fetchCurrentExposure(Metadata *imageMetadata)
{
	std::unique_lock<Metadata> lock(*imageMetadata);
	DeviceStatus *deviceStatus =
		imageMetadata->getLocked<DeviceStatus>("device.status");
	if (!deviceStatus)
		LOG(RPiAgc, Fatal) << "No device metadata";
	current_.shutter = deviceStatus->shutterSpeed;
	current_.analogueGain = deviceStatus->analogueGain;
	AgcStatus *agcStatus =
		imageMetadata->getLocked<AgcStatus>("agc.status");
	current_.totalExposure = agcStatus ? agcStatus->totalExposureValue : 0s;
	current_.totalExposureNoDG = current_.shutter * current_.analogueGain;
}

void Agc::fetchAwbStatus(Metadata *imageMetadata)
{
	awb_.gainR = 1.0; /* in case not found in metadata */
	awb_.gainG = 1.0;
	awb_.gainB = 1.0;
	if (imageMetadata->get("awb.status", awb_) != 0)
		LOG(RPiAgc, Debug) << "No AWB status found";
}

static double computeInitialY(bcm2835_isp_stats *stats, AwbStatus const &awb,
			      double weights[], double gain)
{
	bcm2835_isp_stats_region *regions = stats->agc_stats;
	/*
	 * Note how the calculation below means that equal weights give you
	 * "average" metering (i.e. all pixels equally important).
	 */
	double rSum = 0, gSum = 0, bSum = 0, pixelSum = 0;
	for (unsigned int i = 0; i < AgcStatsSize; i++) {
		double counted = regions[i].counted;
		double rAcc = std::min(regions[i].r_sum * gain, ((1 << PipelineBits) - 1) * counted);
		double gAcc = std::min(regions[i].g_sum * gain, ((1 << PipelineBits) - 1) * counted);
		double bAcc = std::min(regions[i].b_sum * gain, ((1 << PipelineBits) - 1) * counted);
		rSum += rAcc * weights[i];
		gSum += gAcc * weights[i];
		bSum += bAcc * weights[i];
		pixelSum += counted * weights[i];
	}
	if (pixelSum == 0.0) {
		LOG(RPiAgc, Warning) << "computeInitialY: pixelSum is zero";
		return 0;
	}
	double ySum = rSum * awb.gainR * .299 +
		      gSum * awb.gainG * .587 +
		      bSum * awb.gainB * .114;
	return ySum / pixelSum / (1 << PipelineBits);
}

/*
 * We handle extra gain through EV by adjusting our Y targets. However, you
 * simply can't monitor histograms once they get very close to (or beyond!)
 * saturation, so we clamp the Y targets to this value. It does mean that EV
 * increases don't necessarily do quite what you might expect in certain
 * (contrived) cases.
 */

static constexpr double EvGainYTargetLimit = 0.9;

static double constraintComputeGain(AgcConstraint &c, Histogram &h, double lux,
				    double evGain, double &targetY)
{
	targetY = c.yTarget.eval(c.yTarget.domain().clip(lux));
	targetY = std::min(EvGainYTargetLimit, targetY * evGain);
	double iqm = h.interQuantileMean(c.qLo, c.qHi);
	return (targetY * NUM_HISTOGRAM_BINS) / iqm;
}

void Agc::computeGain(bcm2835_isp_stats *statistics, Metadata *imageMetadata,
		      double &gain, double &targetY)
{
	struct LuxStatus lux = {};
	lux.lux = 400; /* default lux level to 400 in case no metadata found */
	if (imageMetadata->get("lux.status", lux) != 0)
		LOG(RPiAgc, Warning) << "No lux level found";
	Histogram h(statistics->hist[0].g_hist, NUM_HISTOGRAM_BINS);
	double evGain = status_.ev * config_.baseEv;
	/*
	 * The initial gain and target_Y come from some of the regions. After
	 * that we consider the histogram constraints.
	 */
	targetY = config_.yTarget.eval(config_.yTarget.domain().clip(lux.lux));
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

void Agc::computeTargetExposure(double gain)
{
	if (status_.fixedShutter && status_.fixedAnalogueGain) {
		/*
		 * When ag and shutter are both fixed, we need to drive the
		 * total exposure so that we end up with a digital gain of at least
		 * 1/minColourGain. Otherwise we'd desaturate channels causing
		 * white to go cyan or magenta.
		 */
		double minColourGain = std::min({ awb_.gainR, awb_.gainG, awb_.gainB, 1.0 });
		ASSERT(minColourGain != 0.0);
		target_.totalExposure =
			status_.fixedShutter * status_.fixedAnalogueGain / minColourGain;
	} else {
		/*
		 * The statistics reflect the image without digital gain, so the final
		 * total exposure we're aiming for is:
		 */
		target_.totalExposure = current_.totalExposureNoDG * gain;
		/* The final target exposure is also limited to what the exposure mode allows. */
		Duration maxShutter = status_.fixedShutter
					      ? status_.fixedShutter
					      : exposureMode_->shutter.back();
		maxShutter = clipShutter(maxShutter);
		Duration maxTotalExposure =
			maxShutter *
			(status_.fixedAnalogueGain != 0.0
				 ? status_.fixedAnalogueGain
				 : exposureMode_->gain.back());
		target_.totalExposure = std::min(target_.totalExposure, maxTotalExposure);
	}
	LOG(RPiAgc, Debug) << "Target totalExposure " << target_.totalExposure;
}

bool Agc::applyDigitalGain(double gain, double targetY)
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
	bool desaturate = targetY > config_.fastReduceThreshold &&
			  gain < sqrt(targetY);
	if (desaturate)
		dg /= config_.fastReduceThreshold;
	LOG(RPiAgc, Debug) << "Digital gain " << dg << " desaturate? " << desaturate;
	target_.totalExposureNoDG = target_.totalExposure / dg;
	LOG(RPiAgc, Debug) << "Target totalExposureNoDG " << target_.totalExposureNoDG;
	return desaturate;
}

void Agc::filterExposure(bool desaturate)
{
	double speed = config_.speed;
	/*
	 * AGC adapts instantly if both shutter and gain are directly specified
	 * or we're in the startup phase.
	 */
	if ((status_.fixedShutter && status_.fixedAnalogueGain) ||
	    frameCount_ <= config_.startupFrames)
		speed = 1.0;
	if (!filtered_.totalExposure) {
		filtered_.totalExposure = target_.totalExposure;
		filtered_.totalExposureNoDG = target_.totalExposureNoDG;
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
		/*
		 * When desaturing, take a big jump down in totalExposureNoDG,
		 * which we'll hide with digital gain.
		 */
		if (desaturate)
			filtered_.totalExposureNoDG =
				target_.totalExposureNoDG;
		else
			filtered_.totalExposureNoDG =
				speed * target_.totalExposureNoDG +
				filtered_.totalExposureNoDG * (1.0 - speed);
	}
	/*
	 * We can't let the totalExposureNoDG exposure deviate too far below the
	 * total exposure, as there might not be enough digital gain available
	 * in the ISP to hide it (which will cause nasty oscillation).
	 */
	if (filtered_.totalExposureNoDG <
	    filtered_.totalExposure * config_.fastReduceThreshold)
		filtered_.totalExposureNoDG = filtered_.totalExposure * config_.fastReduceThreshold;
	LOG(RPiAgc, Debug) << "After filtering, totalExposure " << filtered_.totalExposure
			   << " no dg " << filtered_.totalExposureNoDG;
}

void Agc::divideUpExposure()
{
	/*
	 * Sending the fixed shutter/gain cases through the same code may seem
	 * unnecessary, but it will make more sense when extend this to cover
	 * variable aperture.
	 */
	Duration exposureValue = filtered_.totalExposureNoDG;
	Duration shutterTime;
	double analogueGain;
	shutterTime = status_.fixedShutter ? status_.fixedShutter
					   : exposureMode_->shutter[0];
	shutterTime = clipShutter(shutterTime);
	analogueGain = status_.fixedAnalogueGain != 0.0 ? status_.fixedAnalogueGain
							: exposureMode_->gain[0];
	if (shutterTime * analogueGain < exposureValue) {
		for (unsigned int stage = 1;
		     stage < exposureMode_->gain.size(); stage++) {
			if (!status_.fixedShutter) {
				Duration stageShutter =
					clipShutter(exposureMode_->shutter[stage]);
				if (stageShutter * analogueGain >= exposureValue) {
					shutterTime = exposureValue / analogueGain;
					break;
				}
				shutterTime = stageShutter;
			}
			if (status_.fixedAnalogueGain == 0.0) {
				if (exposureMode_->gain[stage] * shutterTime >= exposureValue) {
					analogueGain = exposureValue / shutterTime;
					break;
				}
				analogueGain = exposureMode_->gain[stage];
			}
		}
	}
	LOG(RPiAgc, Debug) << "Divided up shutter and gain are " << shutterTime << " and "
			   << analogueGain;
	/*
	 * Finally adjust shutter time for flicker avoidance (require both
	 * shutter and gain not to be fixed).
	 */
	if (!status_.fixedShutter && !status_.fixedAnalogueGain &&
	    status_.flickerPeriod) {
		int flickerPeriods = shutterTime / status_.flickerPeriod;
		if (flickerPeriods) {
			Duration newShutterTime = flickerPeriods * status_.flickerPeriod;
			analogueGain *= shutterTime / newShutterTime;
			/*
			 * We should still not allow the ag to go over the
			 * largest value in the exposure mode. Note that this
			 * may force more of the total exposure into the digital
			 * gain as a side-effect.
			 */
			analogueGain = std::min(analogueGain, exposureMode_->gain.back());
			shutterTime = newShutterTime;
		}
		LOG(RPiAgc, Debug) << "After flicker avoidance, shutter "
				   << shutterTime << " gain " << analogueGain;
	}
	filtered_.shutter = shutterTime;
	filtered_.analogueGain = analogueGain;
}

void Agc::writeAndFinish(Metadata *imageMetadata, bool desaturate)
{
	status_.totalExposureValue = filtered_.totalExposure;
	status_.targetExposureValue = desaturate ? 0s : target_.totalExposureNoDG;
	status_.shutterTime = filtered_.shutter;
	status_.analogueGain = filtered_.analogueGain;
	/*
	 * Write to metadata as well, in case anyone wants to update the camera
	 * immediately.
	 */
	imageMetadata->set("agc.status", status_);
	LOG(RPiAgc, Debug) << "Output written, total exposure requested is "
			   << filtered_.totalExposure;
	LOG(RPiAgc, Debug) << "Camera exposure update: shutter time " << filtered_.shutter
			   << " analogue gain " << filtered_.analogueGain;
}

Duration Agc::clipShutter(Duration shutter)
{
	if (maxShutter_)
		shutter = std::min(shutter, maxShutter_);
	return shutter;
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Agc(controller);
}
static RegisterAlgorithm reg(NAME, &create);
