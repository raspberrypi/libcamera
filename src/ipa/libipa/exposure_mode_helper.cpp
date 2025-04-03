/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class that performs computations relating to exposure
 */
#include "exposure_mode_helper.h"

#include <algorithm>

#include <libcamera/base/log.h>

/**
 * \file exposure_mode_helper.h
 * \brief Helper class that performs computations relating to exposure
 *
 * AEGC algorithms have a need to split exposure between exposure time, analogue
 * and digital gain. Multiple implementations do so based on paired stages of
 * exposure time and gain limits; provide a helper to avoid duplicating the code.
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(ExposureModeHelper)

namespace ipa {

/**
 * \class ExposureModeHelper
 * \brief Class for splitting exposure into exposure time and total gain
 *
 * The ExposureModeHelper class provides a standard interface through which an
 * AEGC algorithm can divide exposure between exposure time and gain. It is
 * configured with a set of exposure time and gain pairs and works by initially
 * fixing gain at 1.0 and increasing exposure time up to the exposure time value
 * from the first pair in the set in an attempt to meet the required exposure
 * value.
 *
 * If the required exposure is not achievable by the first exposure time value
 * alone it ramps gain up to the value from the first pair in the set. If the
 * required exposure is still not met it then allows exposure time to ramp up to
 * the exposure time value from the second pair in the set, and continues in this
 * vein until either the required exposure time is met, or else the hardware's
 * exposure time or gain limits are reached.
 *
 * This method allows users to strike a balance between a well-exposed image and
 * an acceptable frame-rate, as opposed to simply maximising exposure time
 * followed by gain. The same helpers can be used to perform the latter
 * operation if needed by passing an empty set of pairs to the initialisation
 * function.
 *
 * The gain values may exceed a camera sensor's analogue gain limits if either
 * it or the IPA is also capable of digital gain. The configure() function must
 * be called with the hardware's limits to inform the helper of those
 * constraints. Any gain that is needed will be applied as analogue gain first
 * until the hardware's limit is reached, following which digital gain will be
 * used.
 */

/**
 * \brief Construct an ExposureModeHelper instance
 * \param[in] stages The vector of paired exposure time and gain limits
 *
 * The input stages are exposure time and _total_ gain pairs; the gain
 * encompasses both analogue and digital gain.
 *
 * The vector of stages may be empty. In that case, the helper will simply use
 * the runtime limits set through setLimits() instead.
 */
ExposureModeHelper::ExposureModeHelper(const Span<std::pair<utils::Duration, double>> stages)
{
	minExposureTime_ = 0us;
	maxExposureTime_ = 0us;
	minGain_ = 0;
	maxGain_ = 0;

	for (const auto &[s, g] : stages) {
		exposureTimes_.push_back(s);
		gains_.push_back(g);
	}
}

/**
 * \brief Set the exposure time and gain limits
 * \param[in] minExposureTime The minimum exposure time supported
 * \param[in] maxExposureTime The maximum exposure time supported
 * \param[in] minGain The minimum analogue gain supported
 * \param[in] maxGain The maximum analogue gain supported
 *
 * This function configures the exposure time and analogue gain limits that need
 * to be adhered to as the helper divides up exposure. Note that this function
 * *must* be called whenever those limits change and before splitExposure() is
 * used.
 *
 * If the algorithm using the helpers needs to indicate that either exposure time
 * or analogue gain or both should be fixed it can do so by setting both the
 * minima and maxima to the same value.
 */
void ExposureModeHelper::setLimits(utils::Duration minExposureTime,
				   utils::Duration maxExposureTime,
				   double minGain, double maxGain)
{
	minExposureTime_ = minExposureTime;
	maxExposureTime_ = maxExposureTime;
	minGain_ = minGain;
	maxGain_ = maxGain;
}

utils::Duration ExposureModeHelper::clampExposureTime(utils::Duration exposureTime) const
{
	return std::clamp(exposureTime, minExposureTime_, maxExposureTime_);
}

double ExposureModeHelper::clampGain(double gain) const
{
	return std::clamp(gain, minGain_, maxGain_);
}

/**
 * \brief Split exposure into exposure time and gain
 * \param[in] exposure Exposure value
 *
 * This function divides a given exposure into exposure time, analogue and
 * digital gain by iterating through stages of exposure time and gain limits.
 * At each stage the current stage's exposure time limit is multiplied by the
 * previous stage's gain limit (or 1.0 initially) to see if the combination of
 * the two can meet the required exposure. If they cannot then the current
 * stage's exposure time limit is multiplied by the same stage's gain limit to
 * see if that combination can meet the required exposure time. If they cannot
 * then the function moves to consider the next stage.
 *
 * When a combination of exposure time and gain _stage_ limits are found that
 * are sufficient to meet the required exposure, the function attempts to reduce
 * exposure time as much as possible whilst fixing gain and still meeting the
 * exposure. If a _runtime_ limit prevents exposure time from being lowered
 * enough to meet the exposure with gain fixed at the stage limit, gain is also
 * lowered to compensate.
 *
 * Once the exposure time and gain values are ascertained, gain is assigned as
 * analogue gain as much as possible, with digital gain only in use if the
 * maximum analogue gain runtime limit is unable to accommodate the exposure
 * value.
 *
 * If no combination of exposure time and gain limits is found that meets the
 * required exposure, the helper falls-back to simply maximising the exposure
 * time first, followed by analogue gain, followed by digital gain.
 *
 * \return Tuple of exposure time, analogue gain, and digital gain
 */
std::tuple<utils::Duration, double, double>
ExposureModeHelper::splitExposure(utils::Duration exposure) const
{
	ASSERT(maxExposureTime_);
	ASSERT(maxGain_);

	bool gainFixed = minGain_ == maxGain_;
	bool exposureTimeFixed = minExposureTime_ == maxExposureTime_;

	/*
	 * There's no point entering the loop if we cannot change either gain
	 * nor exposure time anyway.
	 */
	if (exposureTimeFixed && gainFixed)
		return { minExposureTime_, minGain_, exposure / (minExposureTime_ * minGain_) };

	utils::Duration exposureTime;
	double stageGain = 1.0;
	double gain;

	for (unsigned int stage = 0; stage < gains_.size(); stage++) {
		double lastStageGain = stage == 0 ? 1.0 : clampGain(gains_[stage - 1]);
		utils::Duration stageExposureTime = clampExposureTime(exposureTimes_[stage]);
		stageGain = clampGain(gains_[stage]);

		/*
		 * We perform the clamping on both exposure time and gain in
		 * case the helper has had limits set that prevent those values
		 * being lowered beyond a certain minimum...this can happen at
		 * runtime for various reasons and so would not be known when
		 * the stage limits are initialised.
		 */

		/* Clamp the gain to lastStageGain and regulate exposureTime. */
		if (stageExposureTime * lastStageGain >= exposure) {
			exposureTime = clampExposureTime(exposure / clampGain(lastStageGain));
			gain = clampGain(exposure / exposureTime);

			return { exposureTime, gain, exposure / (exposureTime * gain) };
		}

		/* Clamp the exposureTime to stageExposureTime and regulate gain. */
		if (stageExposureTime * stageGain >= exposure) {
			exposureTime = clampExposureTime(stageExposureTime);
			gain = clampGain(exposure / exposureTime);

			return { exposureTime, gain, exposure / (exposureTime * gain) };
		}
	}

	/*
	 * From here on all we can do is max out the exposure time, followed by
	 * the analogue gain. If we still haven't achieved the target we send
	 * the rest of the exposure time to digital gain. If we were given no
	 * stages to use then the default stageGain of 1.0 is used so that
	 * exposure time is maxed before gain is touched at all.
	 */
	exposureTime = clampExposureTime(exposure / clampGain(stageGain));
	gain = clampGain(exposure / exposureTime);

	return { exposureTime, gain, exposure / (exposureTime * gain) };
}

/**
 * \fn ExposureModeHelper::minExposureTime()
 * \brief Retrieve the configured minimum exposure time limit set through
 * setLimits()
 * \return The minExposureTime_ value
 */

/**
 * \fn ExposureModeHelper::maxExposureTime()
 * \brief Retrieve the configured maximum exposure time set through setLimits()
 * \return The maxExposureTime_ value
 */

/**
 * \fn ExposureModeHelper::minGain()
 * \brief Retrieve the configured minimum gain set through setLimits()
 * \return The minGain_ value
 */

/**
 * \fn ExposureModeHelper::maxGain()
 * \brief Retrieve the configured maximum gain set through setLimits()
 * \return The maxGain_ value
 */

} /* namespace ipa */

} /* namespace libcamera */
