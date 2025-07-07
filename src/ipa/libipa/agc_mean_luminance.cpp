/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Base class for mean luminance AGC algorithms
 */

#include "agc_mean_luminance.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>

#include "exposure_mode_helper.h"

using namespace libcamera::controls;

/**
 * \file agc_mean_luminance.h
 * \brief Base class implementing mean luminance AEGC
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
 * should derive it and provide an overriding estimateLuminance() function for
 * this class to use. They must call parseTuningData() in init(), and must also
 * call setLimits() and resetFrameCounter() in configure(). They may then use
 * calculateNewEv() in process(). If the limits passed to setLimits() change for
 * any reason (for example, in response to a FrameDurationLimit control being
 * passed in queueRequest()) then setLimits() must be called again with the new
 * values.
 */

AgcMeanLuminance::AgcMeanLuminance()
	: frameCount_(0), filteredExposure_(0s), relativeLuminanceTarget_(0)
{
}

AgcMeanLuminance::~AgcMeanLuminance() = default;

void AgcMeanLuminance::parseRelativeLuminanceTarget(const YamlObject &tuningData)
{
	relativeLuminanceTarget_ =
		tuningData["relativeLuminanceTarget"].get<double>(kDefaultRelativeLuminanceTarget);
}

void AgcMeanLuminance::parseConstraint(const YamlObject &modeDict, int32_t id)
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
		double yTarget =
			content["yTarget"].getList<double>().value_or(std::vector<double>{ 0.5 }).at(0);

		AgcConstraint constraint = { bound, qLo, qHi, yTarget };

		if (!constraintModes_.count(id))
			constraintModes_[id] = {};

		if (idx)
			constraintModes_[id].push_back(constraint);
		else
			constraintModes_[id].insert(constraintModes_[id].begin(), constraint);
	}
}

int AgcMeanLuminance::parseConstraintModes(const YamlObject &tuningData)
{
	std::vector<ControlValue> availableConstraintModes;

	const YamlObject &yamlConstraintModes = tuningData[controls::AeConstraintMode.name()];
	if (yamlConstraintModes.isDictionary()) {
		for (const auto &[modeName, modeDict] : yamlConstraintModes.asDict()) {
			if (AeConstraintModeNameValueMap.find(modeName) ==
			    AeConstraintModeNameValueMap.end()) {
				LOG(AgcMeanLuminance, Warning)
					<< "Skipping unknown constraint mode '" << modeName << "'";
				continue;
			}

			if (!modeDict.isDictionary()) {
				LOG(AgcMeanLuminance, Error)
					<< "Invalid constraint mode '" << modeName << "'";
				return -EINVAL;
			}

			parseConstraint(modeDict,
					AeConstraintModeNameValueMap.at(modeName));
			availableConstraintModes.push_back(
				AeConstraintModeNameValueMap.at(modeName));
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
			0.5
		};

		constraintModes_[controls::ConstraintNormal].insert(
			constraintModes_[controls::ConstraintNormal].begin(),
			constraint);
		availableConstraintModes.push_back(controls::ConstraintNormal);
	}

	controls_[&controls::AeConstraintMode] = ControlInfo(availableConstraintModes);

	return 0;
}

int AgcMeanLuminance::parseExposureModes(const YamlObject &tuningData)
{
	std::vector<ControlValue> availableExposureModes;

	const YamlObject &yamlExposureModes = tuningData[controls::AeExposureMode.name()];
	if (yamlExposureModes.isDictionary()) {
		for (const auto &[modeName, modeValues] : yamlExposureModes.asDict()) {
			if (AeExposureModeNameValueMap.find(modeName) ==
			    AeExposureModeNameValueMap.end()) {
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
				modeValues["exposureTime"].getList<uint32_t>().value_or(std::vector<uint32_t>{});
			std::vector<double> gains =
				modeValues["gain"].getList<double>().value_or(std::vector<double>{});

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

			std::shared_ptr<ExposureModeHelper> helper =
				std::make_shared<ExposureModeHelper>(stages);

			exposureModeHelpers_[AeExposureModeNameValueMap.at(modeName)] = helper;
			availableExposureModes.push_back(AeExposureModeNameValueMap.at(modeName));
		}
	}

	/*
	 * If we don't have any exposure modes in the tuning data we create an
	 * ExposureModeHelper using an empty vector of stages. This will result
	 * in the ExposureModeHelper simply driving the exposure time as high as
	 * possible before touching gain.
	 */
	if (availableExposureModes.empty()) {
		int32_t exposureModeId = controls::ExposureNormal;
		std::vector<std::pair<utils::Duration, double>> stages = { };

		std::shared_ptr<ExposureModeHelper> helper =
			std::make_shared<ExposureModeHelper>(stages);

		exposureModeHelpers_[exposureModeId] = helper;
		availableExposureModes.push_back(exposureModeId);
	}

	controls_[&controls::AeExposureMode] = ControlInfo(availableExposureModes);

	return 0;
}

/**
 * \brief Parse tuning data for AeConstraintMode and AeExposureMode controls
 * \param[in] tuningData the YamlObject representing the tuning data
 *
 * This function parses tuning data to build the list of allowed values for the
 * AeConstraintMode and AeExposureMode controls. Those tuning data must provide
 * the data in a specific format; the Agc algorithm's tuning data should contain
 * a dictionary called AeConstraintMode containing per-mode setting dictionaries
 * with the key being a value from \ref controls::AeConstraintModeNameValueMap.
 * Each mode dict may contain either a "lower" or "upper" key or both, for
 * example:
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
 *             yTarget: 0.8
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
int AgcMeanLuminance::parseTuningData(const YamlObject &tuningData)
{
	int ret;

	parseRelativeLuminanceTarget(tuningData);

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
 *
 * This function calls \ref ExposureModeHelper::setLimits() for each
 * ExposureModeHelper that has been created for this class.
 */
void AgcMeanLuminance::setLimits(utils::Duration minExposureTime,
				 utils::Duration maxExposureTime,
				 double minGain, double maxGain)
{
	for (auto &[id, helper] : exposureModeHelpers_)
		helper->setLimits(minExposureTime, maxExposureTime, minGain, maxGain);
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
 * \fn AgcMeanLuminance::controls()
 * \brief Get the controls that have been generated after parsing tuning data
 */

/**
 * \fn AgcMeanLuminance::estimateLuminance(const double gain)
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
 * \brief Estimate the initial gain needed to achieve a relative luminance
 * target
 * \return The calculated initial gain
 */
double AgcMeanLuminance::estimateInitialGain() const
{
	double yTarget = relativeLuminanceTarget_;
	double yGain = 1.0;

	/*
	* To account for non-linearity caused by saturation, the value needs to
	* be estimated in an iterative process, as multiplying by a gain will
	* not increase the relative luminance by the same factor if some image
	* regions are saturated.
	*/
	for (unsigned int i = 0; i < 8; i++) {
		double yValue = estimateLuminance(yGain);
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
 * \param[in] constraintModeIndex The index of the constraint to adhere to
 * \param[in] hist A histogram over which to calculate inter-quantile means
 * \param[in] gain The gain to clamp
 *
 * \return The gain clamped within the constraint bounds
 */
double AgcMeanLuminance::constraintClampGain(uint32_t constraintModeIndex,
					     const Histogram &hist,
					     double gain)
{
	std::vector<AgcConstraint> &constraints = constraintModes_[constraintModeIndex];
	for (const AgcConstraint &constraint : constraints) {
		double newGain = constraint.yTarget * hist.bins() /
				 hist.interQuantileMean(constraint.qLo, constraint.qHi);

		if (constraint.bound == AgcConstraint::Bound::Lower &&
		    newGain > gain)
			gain = newGain;

		if (constraint.bound == AgcConstraint::Bound::Upper &&
		    newGain < gain)
			gain = newGain;
	}

	return gain;
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
 * \brief Calculate the new exposure value and splut it between exposure time
 * and gain
 * \param[in] constraintModeIndex The index of the current constraint mode
 * \param[in] exposureModeIndex The index of the current exposure mode
 * \param[in] yHist A Histogram from the ISP statistics to use in constraining
 * the calculated gain
 * \param[in] effectiveExposureValue The EV applied to the frame from which the
 * statistics in use derive
 *
 * Calculate a new exposure value to try to obtain the target. The calculated
 * exposure value is filtered to prevent rapid changes from frame to frame, and
 * divided into exposure time, analogue and digital gain.
 *
 * \return Tuple of exposure time, analogue gain, and digital gain
 */
std::tuple<utils::Duration, double, double>
AgcMeanLuminance::calculateNewEv(uint32_t constraintModeIndex,
				 uint32_t exposureModeIndex,
				 const Histogram &yHist,
				 utils::Duration effectiveExposureValue)
{
	/*
	 * The pipeline handler should validate that we have received an allowed
	 * value for AeExposureMode.
	 */
	std::shared_ptr<ExposureModeHelper> exposureModeHelper =
		exposureModeHelpers_.at(exposureModeIndex);

	if (effectiveExposureValue == 0s) {
		LOG(AgcMeanLuminance, Error)
			<< "Effective exposure value is 0. This is a bug in AGC "
			   "and must be fixed for proper operation.";
		/*
		 * Return an arbitrary exposure time > 0 to ensure regulation
		 * doesn't get stuck with 0 in case the sensor driver allows a
		 * min exposure of 0.
		 */
		return exposureModeHelper->splitExposure(10ms);
	}

	double gain = estimateInitialGain();
	gain = constraintClampGain(constraintModeIndex, yHist, gain);

	/*
	 * We don't check whether we're already close to the target, because
	 * even if the effective exposure value is the same as the last frame's
	 * we could have switched to an exposure mode that would require a new
	 * pass through the splitExposure() function.
	 */

	utils::Duration newExposureValue = effectiveExposureValue * gain;

	/*
	 * We filter the exposure value to make sure changes are not too jarring
	 * from frame to frame.
	 */
	newExposureValue = filterExposure(newExposureValue);

	frameCount_++;
	return exposureModeHelper->splitExposure(newExposureValue);
}

/**
 * \fn AgcMeanLuminance::resetFrameCount()
 * \brief Reset the frame counter
 *
 * This function resets the internal frame counter, which exists to help the
 * algorithm decide whether it should respond instantly or not. The expectation
 * is for derived classes to call this function before each camera start call in
 * their configure() function.
 */

} /* namespace ipa */

} /* namespace libcamera */
