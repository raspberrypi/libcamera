/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Base class for mean luminance AGC algorithms
 */

#include "agc_mean_luminance.h"

#include <algorithm>
#include <cmath>
#include <span>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "exposure_mode_helper.h"

using namespace libcamera::controls;

/**
 * \file agc_mean_luminance.h
 * \brief Class implementing mean luminance AEGC
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(AgcMeanLuminance)

namespace ipa {

/*
 * Number of frames for which to run the algorithm at full speed, before slowing
 * down to prevent large and jarring changes in exposure from frame to frame.
 */
static constexpr uint32_t kNumStartupFrames = 10;

/*
 * Default relative luminance target
 *
 * This value should be chosen so that when the camera points at a grey target,
 * the resulting image brightness looks "right". Custom values can be passed
 * as the relativeLuminanceTarget value in sensor tuning files.
 */
static constexpr double kDefaultRelativeLuminanceTarget = 0.16;

/*
 * Maximum relative luminance target
 *
 * This value limits the relative luminance target after applying the exposure
 * compensation. Targeting a value above this limit results in saturation
 * and the inability to regulate properly.
 */
static constexpr double kMaxRelativeLuminanceTarget = 0.95;

/*
 * Default lux level
 *
 * If no lux level or a zero lux level is specified, but PWLs are used to
 * specify luminance targets, this default level is used.
 */
static constexpr unsigned int kDefaultLuxLevel = 500;

/**
 * \struct AgcMeanLuminance::AgcConstraint
 * \brief The boundaries and target for an AeConstraintMode constraint
 *
 * This structure describes an AeConstraintMode constraint for the purposes of
 * this algorithm. These constraints are expressed as a pair of quantile
 * boundaries for a histogram, along with a luminance target and a bounds-type.
 * The algorithm uses the constraints by ensuring that the defined portion of a
 * luminance histogram (I.E. lying between the two quantiles) is above or below
 * the given luminance value.
 */

/**
 * \enum AgcMeanLuminance::AgcConstraint::Bound
 * \brief Specify whether the constraint defines a lower or upper bound
 * \var AgcMeanLuminance::AgcConstraint::Lower
 * \brief The constraint defines a lower bound
 * \var AgcMeanLuminance::AgcConstraint::Upper
 * \brief The constraint defines an upper bound
 */

/**
 * \var AgcMeanLuminance::AgcConstraint::bound
 * \brief The type of constraint bound
 */

/**
 * \var AgcMeanLuminance::AgcConstraint::qLo
 * \brief The lower quantile to use for the constraint
 */

/**
 * \var AgcMeanLuminance::AgcConstraint::qHi
 * \brief The upper quantile to use for the constraint
 */

/**
 * \var AgcMeanLuminance::AgcConstraint::yTarget
 * \brief The luminance target for the constraint
 */

/**
 * \class AgcMeanLuminance::Traits
 * \brief A collection of callbacks
 *
 * This type contains virtual methods that provide the necessary pieces of
 * information for the algorithm, and are to be implemented by the user.
 */

/**
 * \fn AgcMeanLuminance::Traits::estimateLuminance(double gain)
 * \brief Estimate the luminance of an image, adjusted by a given gain
 * \param[in] gain The gain with which to adjust the luminance estimate
 *
 * This function estimates the average relative luminance of the frame that
 * would be output by the sensor if an additional \a gain was applied. It is a
 * pure virtual function because estimation of luminance is a hardware-specific
 * operation, which depends wholly on the format of the stats that are delivered
 * to libcamera from the ISP. Derived classes must override this function with
 * one that calculates the normalised mean luminance value across the entire
 * image.
 *
 * \return The normalised relative luminance of the image
 */

/**
 * \class AgcMeanLuminance
 * \brief A mean-based auto-exposure algorithm
 *
 * This algorithm calculates an exposure time, analogue and digital gain such
 * that the normalised mean luminance value of an image is driven towards a
 * target, which itself is discovered from tuning data. The algorithm is a
 * two-stage process.
 *
 * In the first stage, an initial gain value is derived by iteratively comparing
 * the gain-adjusted mean luminance across the entire image against a target,
 * and selecting a value which pushes it as closely as possible towards the
 * target.
 *
 * In the second stage we calculate the gain required to drive the average of a
 * section of a histogram to a target value, where the target and the boundaries
 * of the section of the histogram used in the calculation are taken from the
 * values defined for the currently configured AeConstraintMode within the
 * tuning data. This class provides a helper function to parse those tuning data
 * to discover the constraints, and so requires a specific format for those
 * data which is described in \ref parseTuningData(). The gain from the first
 * stage is then clamped to the gain from this stage.
 *
 * The final gain is used to adjust the effective exposure value of the image,
 * and that new exposure value is divided into exposure time, analogue gain and
 * digital gain according to the selected AeExposureMode. This class uses the
 * \ref ExposureModeHelper class to assist in that division, and expects the
 * data needed to initialise that class to be present in tuning data in a
 * format described in \ref parseTuningData().
 *
 * In order to be able to use this algorithm an IPA module needs to be able to
 * do the following:
 *
 * 1. Provide a luminance estimation across an entire image.
 * 2. Provide a luminance Histogram for the image to use in calculating
 *    constraint compliance. The precision of the Histogram that is available
 *    will determine the supportable precision of the constraints.
 *
 * IPA modules that want to use this class to implement their AEGC algorithm
 * should derive AgcMeanLuminance::Traits and override the necessary functions.
 * The users must call parseTuningData() in init(), and must also call configure()
 * and setLimits() in configure(). They may then use calculateNewEv() in process().
 * If the limits passed to setLimits() change for any reason (for example, in
 * response to a FrameDurationLimit control being passed in queueRequest()) then
 * setLimits() must be called again with the new values.
 */

AgcMeanLuminance::AgcMeanLuminance()
	: filteredExposure_(0s), luxWarningEnabled_(true), frameCount_(0)
{
}

AgcMeanLuminance::~AgcMeanLuminance() = default;

int AgcMeanLuminance::parseRelativeLuminanceTarget(const ValueNode &tuningData)
{
	auto &target = tuningData["relativeLuminanceTarget"];
	if (!target) {
		relativeLuminanceTarget_ = { { { { 0.0, kDefaultRelativeLuminanceTarget } } } };
		return 0;
	}

	auto pwl = target.get<Pwl>();
	if (!pwl) {
		LOG(AgcMeanLuminance, Error)
			<< "Failed to load relative luminance target.";
		return -EINVAL;
	}

	relativeLuminanceTarget_ = std::move(*pwl);
	return 0;
}

int AgcMeanLuminance::parseConstraint(const ValueNode &modeDict, int32_t id)
{
	for (const auto &[boundName, content] : modeDict.asDict()) {
		if (boundName != "upper" && boundName != "lower") {
			LOG(AgcMeanLuminance, Warning)
				<< "Ignoring unknown constraint bound '" << boundName << "'";
			continue;
		}

		unsigned int idx = static_cast<unsigned int>(boundName == "upper");
		AgcConstraint::Bound bound = static_cast<AgcConstraint::Bound>(idx);
		double qLo = content["qLo"].get<double>().value_or(0.98);
		double qHi = content["qHi"].get<double>().value_or(1.0);
		auto yTarget = content["yTarget"].get<Pwl>();
		if (!yTarget) {
			LOG(AgcMeanLuminance, Error)
				<< "Failed to parse yTarget";
			return -EINVAL;
		}

		AgcConstraint constraint = { bound, qLo, qHi, std::move(*yTarget) };
		auto &constraints = constraintModes_[id];

		if (idx)
			constraints.push_back(std::move(constraint));
		else
			constraints.insert(constraints.begin(), std::move(constraint));
	}

	return 0;
}

int AgcMeanLuminance::parseConstraintModes(const ValueNode &tuningData)
{
	const ValueNode &constraintModes = tuningData[controls::AeConstraintMode.name()];
	if (constraintModes.isDictionary()) {
		for (const auto &[modeName, modeDict] : constraintModes.asDict()) {
			auto it = AeConstraintModeNameValueMap.find(modeName);
			if (it == AeConstraintModeNameValueMap.end()) {
				LOG(AgcMeanLuminance, Warning)
					<< "Skipping unknown constraint mode '" << modeName << "'";
				continue;
			}

			if (!modeDict.isDictionary()) {
				LOG(AgcMeanLuminance, Error)
					<< "Invalid constraint mode '" << modeName << "'";
				return -EINVAL;
			}

			int ret = parseConstraint(modeDict, it->second);
			if (ret)
				return ret;
		}
	}

	/*
	 * If the tuning data file contains no constraints then we use the
	 * default constraint that the IPU3/RkISP1 Agc algorithms were adhering
	 * to anyway before centralisation; this constraint forces the top 2% of
	 * the histogram to be at least 0.5.
	 */
	if (constraintModes_.empty()) {
		AgcConstraint constraint = {
			AgcConstraint::Bound::Lower,
			0.98,
			1.0,
			Pwl({ { { 0.0, 0.5 } } })
		};

		constraintModes_[controls::ConstraintNormal].push_back(std::move(constraint));
	}

	return 0;
}

int AgcMeanLuminance::parseExposureModes(const ValueNode &tuningData)
{
	const ValueNode &exposureModes = tuningData[controls::AeExposureMode.name()];
	if (exposureModes.isDictionary()) {
		for (const auto &[modeName, modeValues] : exposureModes.asDict()) {
			auto it = AeExposureModeNameValueMap.find(modeName);
			if (it == AeExposureModeNameValueMap.end()) {
				LOG(AgcMeanLuminance, Warning)
					<< "Skipping unknown exposure mode '" << modeName << "'";
				continue;
			}

			if (!modeValues.isDictionary()) {
				LOG(AgcMeanLuminance, Error)
					<< "Invalid exposure mode '" << modeName << "'";
				return -EINVAL;
			}

			std::vector<uint32_t> exposureTimes =
				modeValues["exposureTime"].get<std::vector<uint32_t>>().value_or(utils::defopt);
			std::vector<double> gains =
				modeValues["gain"].get<std::vector<double>>().value_or(utils::defopt);

			if (exposureTimes.size() != gains.size()) {
				LOG(AgcMeanLuminance, Error)
					<< "Exposure time and gain array sizes unequal";
				return -EINVAL;
			}

			if (exposureTimes.empty()) {
				LOG(AgcMeanLuminance, Error)
					<< "Exposure time and gain arrays are empty";
				return -EINVAL;
			}

			std::vector<std::pair<utils::Duration, double>> stages;
			for (unsigned int i = 0; i < exposureTimes.size(); i++) {
				stages.push_back({
					std::chrono::microseconds(exposureTimes[i]),
					gains[i]
				});
			}

			exposureModeHelpers_.try_emplace(it->second, stages);
		}
	}

	/*
	 * If we don't have any exposure modes in the tuning data we create an
	 * ExposureModeHelper using an empty vector of stages. This will result
	 * in the ExposureModeHelper simply driving the exposure time as high as
	 * possible before touching gain.
	 */
	if (exposureModeHelpers_.empty())
		exposureModeHelpers_.try_emplace(controls::ExposureNormal,
						 std::span<std::pair<utils::Duration, double>>{});

	return 0;
}

/**
 * \brief Configure the exposure mode helpers
 * \param[in] lineDuration The sensor line length
 * \param[in] sensorHelper The sensor helper
 *
 * This function configures the exposure mode helpers so they can correctly
 * take quantization effects into account.
 */
void AgcMeanLuminance::configure(utils::Duration lineDuration,
				 const CameraSensorHelper *sensorHelper)
{
	for (auto &[id, helper] : exposureModeHelpers_)
		helper.configure(lineDuration, sensorHelper);

	luxWarningEnabled_ = true;
	filteredExposure_ = 0s;
	frameCount_ = 0;
}

/**
 * \brief Parse tuning data for AeConstraintMode and AeExposureMode controls
 * \param[in] tuningData the ValueNode representing the tuning data
 *
 * This function parses tuning data to build the list of allowed values for the
 * AeConstraintMode and AeExposureMode controls. Those tuning data must provide
 * the data in a specific format; the Agc algorithm's tuning data should contain
 * a dictionary called AeConstraintMode containing per-mode setting dictionaries
 * with the key being a value from \ref controls::AeConstraintModeNameValueMap.
 * The yTarget can either be provided as single value or as array in which case
 * it is interpreted as a PWL mapping lux levels to yTarget values. Each mode
 * dict may contain either a "lower" or "upper" key or both, for example:
 *
 * \code{.unparsed}
 * algorithms:
 *   - Agc:
 *       AeConstraintMode:
 *         ConstraintNormal:
 *           lower:
 *             qLo: 0.98
 *             qHi: 1.0
 *             yTarget: 0.5
 *         ConstraintHighlight:
 *           lower:
 *             qLo: 0.98
 *             qHi: 1.0
 *             yTarget: 0.5
 *           upper:
 *             qLo: 0.98
 *             qHi: 1.0
 *             yTarget: [ 100, 0.8, 20000, 0.5 ]
 *
 * \endcode
 *
 * For the AeExposureMode control the data should contain a dictionary called
 * AeExposureMode containing per-mode setting dictionaries with the key being a
 * value from \ref controls::AeExposureModeNameValueMap. Each mode dict should
 * contain an array of exposure times with the key "exposureTime" and an array
 * of gain values with the key "gain", in this format:
 *
 * \code{.unparsed}
 * algorithms:
 *   - Agc:
 *       AeExposureMode:
 *         ExposureNormal:
 *           exposureTime: [ 100, 10000, 30000, 60000, 120000 ]
 *           gain: [ 2.0, 4.0, 6.0, 8.0, 10.0 ]
 *         ExposureShort:
 *           exposureTime: [ 100, 10000, 30000, 60000, 120000 ]
 *           gain: [ 2.0, 4.0, 6.0, 8.0, 10.0 ]
 *
 * \endcode
 *
 * \return 0 on success or a negative error code
 */
int AgcMeanLuminance::parseTuningData(const ValueNode &tuningData)
{
	int ret;

	ret = parseRelativeLuminanceTarget(tuningData);
	if (ret)
		return ret;

	ret = parseConstraintModes(tuningData);
	if (ret)
		return ret;

	return parseExposureModes(tuningData);
}

/**
 * \brief Set the ExposureModeHelper limits for this class
 * \param[in] minExposureTime Minimum exposure time to allow
 * \param[in] maxExposureTime Maximum ewposure time to allow
 * \param[in] minGain Minimum gain to allow
 * \param[in] maxGain Maximum gain to allow
 * \param[in] constraints Additional constraints to apply
 *
 * This function calls \ref ExposureModeHelper::setLimits() for each
 * ExposureModeHelper that has been created for this class.
 */
void AgcMeanLuminance::setLimits(utils::Duration minExposureTime,
				 utils::Duration maxExposureTime,
				 double minGain, double maxGain,
				 std::vector<AgcMeanLuminance::AgcConstraint> constraints)
{
	for (auto &[id, helper] : exposureModeHelpers_)
		helper.setLimits(minExposureTime, maxExposureTime, minGain, maxGain);

	additionalConstraints_ = std::move(constraints);
}

/**
 * \fn AgcMeanLuminance::constraintModes()
 * \brief Get the constraint modes that have been parsed from tuning data
 */

/**
 * \fn AgcMeanLuminance::exposureModeHelpers()
 * \brief Get the ExposureModeHelpers that have been parsed from tuning data
 */

/**
 * \brief Estimate the initial gain needed to achieve a relative luminance
 * target
 * \return The calculated initial gain
 */
double AgcMeanLuminance::estimateInitialGain(const Traits &traits, double yTarget) const
{
	double yGain = 1.0;

	/*
	* To account for non-linearity caused by saturation, the value needs to
	* be estimated in an iterative process, as multiplying by a gain will
	* not increase the relative luminance by the same factor if some image
	* regions are saturated.
	*/
	for (unsigned int i = 0; i < 8; i++) {
		double yValue = traits.estimateLuminance(yGain);
		double extra_gain = std::min(10.0, yTarget / (yValue + .001));

		yGain *= extra_gain;
		LOG(AgcMeanLuminance, Debug) << "Y value: " << yValue
				<< ", Y target: " << yTarget
				<< ", gives gain " << yGain;

		if (utils::abs_diff(extra_gain, 1.0) < 0.01)
			break;
	}

	return yGain;
}

/**
 * \brief Clamp gain within the bounds of a defined constraint
 * \param[in] params The set of parameters for the calculation
 * \param[in] gain The gain to clamp
 *
 * \return The gain clamped within the constraint bounds
 */
double AgcMeanLuminance::constraintClampGain(const Params &params, double gain) const
{
	auto applyConstraint = [&](const AgcConstraint &constraint) {
		double lux = params.lux;

		if (relativeLuminanceTarget_.size() > 1 && lux == 0)
			lux = kDefaultLuxLevel;

		double target = constraint.yTarget.eval(
			constraint.yTarget.domain().clamp(lux));
		double newGain = target * params.yHist.bins() /
				 params.yHist.interQuantileMean(constraint.qLo, constraint.qHi);

		if (constraint.bound == AgcConstraint::Bound::Lower &&
		    newGain > gain) {
			LOG(AgcMeanLuminance, Debug)
				<< "Apply lower bound: " << gain << " to "
				<< newGain;
			gain = newGain;
		}

		if (constraint.bound == AgcConstraint::Bound::Upper &&
		    newGain < gain) {
			LOG(AgcMeanLuminance, Debug)
				<< "Apply upper bound: " << gain << " to "
				<< newGain;
			gain = newGain;
		}
	};

	const std::vector<AgcConstraint> &constraints = constraintModes_.at(params.constraintModeIndex);
	std::for_each(constraints.begin(), constraints.end(), applyConstraint);

	std::for_each(additionalConstraints_.begin(), additionalConstraints_.end(), applyConstraint);

	return gain;
}

/**
 * \brief Get the currently effective y target
 * \param[in] lux The effective lux value
 * \param[in] exposureCompensation The exposure compensation value
 *
 * This function returns the current y target including exposure compensation.
 *
 * \return The y target value
 */
double AgcMeanLuminance::effectiveYTarget(double lux, double exposureCompensation) const
{
	if (relativeLuminanceTarget_.size() > 1 && lux == 0) {
		/*
		 * Warn after a few frames if there is still no lux measurement
		 * available. The number of 10 is chosen a bit arbitrarily. It
		 * is big enough to skip the frames that get queued on start
		 * (and therefore are expected to have no valid lux value) and
		 * small enough to show up quickly.
		 */
		if (frameCount_ > 10 && luxWarningEnabled_) {
			luxWarningEnabled_ = false;
			LOG(AgcMeanLuminance, Warning)
				<< "Missing lux value for luminance target "
				   "calculation, default to "
				<< kDefaultLuxLevel
				<< ". Note that the Lux algorithm must be "
				   "included before the Agc algorithm.";
		}

		lux = kDefaultLuxLevel;
	}

	double luminanceTarget = relativeLuminanceTarget_.eval(
		relativeLuminanceTarget_.domain().clamp(lux));

	return std::min(luminanceTarget * exposureCompensation,
			kMaxRelativeLuminanceTarget);
}

/**
 * \brief Apply a filter on the exposure value to limit the speed of changes
 * \param[in] exposureValue The target exposure from the AGC algorithm
 *
 * The speed of the filter is adaptive, and will produce the target quicker
 * during startup, or when the target exposure is within 20% of the most recent
 * filter output.
 *
 * \return The filtered exposure
 */
utils::Duration AgcMeanLuminance::filterExposure(utils::Duration exposureValue)
{
	double speed = 0.2;

	/* Adapt instantly if we are in startup phase. */
	if (frameCount_ < kNumStartupFrames)
		speed = 1.0;

	/*
	 * If we are close to the desired result, go faster to avoid making
	 * multiple micro-adjustments.
	 * \todo Make this customisable?
	 */
	if (filteredExposure_ < 1.2 * exposureValue &&
	    filteredExposure_ > 0.8 * exposureValue)
		speed = sqrt(speed);

	filteredExposure_ = speed * exposureValue +
			    filteredExposure_ * (1.0 - speed);

	return filteredExposure_;
}

/**
 * \struct AgcMeanLuminance::Params
 * \brief Collection of parameters for the mean luminance AGC algorithm
 *
 * \var AgcMeanLuminance::Params::traits
 * \brief The traits object implementing the necessary functions
 *
 * \var AgcMeanLuminance::Params::yHist
 * \brief A histogram from the ISP statistics to use in constraining the calculated gain
 *
 * \var AgcMeanLuminance::Params::effectiveExposureValue
 * \brief The EV applied to the frame from which the statistics in use derive
 *
 * \var AgcMeanLuminance::Params::constraintModeIndex
 * \brief The index of the constraint mode to use
 *
 * \var AgcMeanLuminance::Params::exposureModeIndex
 * \brief The index of the exposure mode to use
 *
 * \var AgcMeanLuminance::Params::lux
 * \brief The lux level to be used in the AGC calculations. A value of 0 means no
 * measurement and a default value of \a kDefaultLuxLevel is used if necessary.
 *
 * \var AgcMeanLuminance::Params::exposureCompensation
 * \brief The exposure compensation value to be used in the AGC calculations
 * It is expressed as gain instead of EV.
 */

/**
 * \struct AgcMeanLuminance::Result
 * \brief Collection of results of the mean luminance AGC algorithm
 *
 * \var AgcMeanLuminance::Result::yTarget
 * \brief The current y target including exposure compensation
 */

/**
 * \brief Calculate the new exposure value and split it between exposure time
 * and gain
 * \param[in] params The set of parameters for the exppsure value calculation
 *
 * Calculate a new exposure value to try to obtain the target. The calculated
 * exposure value is filtered to prevent rapid changes from frame to frame, and
 * divided into exposure time, analogue, quantization and digital gain.
 *
 * \return Tuple of exposure time, analogue gain, quantization gain and digital
 * gain
 */
AgcMeanLuminance::Result
AgcMeanLuminance::calculateNewEv(const Params &params)
{
	/*
	 * The pipeline handler should validate that we have received an allowed
	 * value for AeExposureMode.
	 */
	ExposureModeHelper &exposureModeHelper =
		exposureModeHelpers_.at(params.exposureModeIndex);
	double yTarget = effectiveYTarget(params.lux, params.exposureCompensation);

	if (params.effectiveExposureValue == 0s) {
		LOG(AgcMeanLuminance, Error)
			<< "Effective exposure value is 0. This is a bug in AGC "
			   "and must be fixed for proper operation.";
		/*
		 * Return an arbitrary exposure time > 0 to ensure regulation
		 * doesn't get stuck with 0 in case the sensor driver allows a
		 * min exposure of 0.
		 */
		return { exposureModeHelper.splitExposure(10ms), yTarget };
	}

	double gain = estimateInitialGain(params.traits, yTarget);
	gain = constraintClampGain(params, gain);

	/*
	 * We don't check whether we're already close to the target, because
	 * even if the effective exposure value is the same as the last frame's
	 * we could have switched to an exposure mode that would require a new
	 * pass through the splitExposure() function.
	 */

	utils::Duration newExposureValue = params.effectiveExposureValue * gain;

	/*
	 * We filter the exposure value to make sure changes are not too jarring
	 * from frame to frame.
	 */
	newExposureValue = filterExposure(newExposureValue);

	frameCount_++;
	return { exposureModeHelper.splitExposure(newExposureValue), yTarget };
}

} /* namespace ipa */

} /* namespace libcamera */
