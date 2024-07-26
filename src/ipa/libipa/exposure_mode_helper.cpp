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
 * AEGC algorithms have a need to split exposure between shutter time, analogue
 * and digital gain. Multiple implementations do so based on paired stages of
 * shutter time and gain limits; provide a helper to avoid duplicating the code.
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(ExposureModeHelper)

namespace ipa {

/**
 * \class ExposureModeHelper
 * \brief Class for splitting exposure into shutter time and total gain
 *
 * The ExposureModeHelper class provides a standard interface through which an
 * AEGC algorithm can divide exposure between shutter time and gain. It is
 * configured with a set of shutter time and gain pairs and works by initially
 * fixing gain at 1.0 and increasing shutter time up to the shutter time value
 * from the first pair in the set in an attempt to meet the required exposure
 * value.
 *
 * If the required exposure is not achievable by the first shutter time value
 * alone it ramps gain up to the value from the first pair in the set. If the
 * required exposure is still not met it then allows shutter time to ramp up to
 * the shutter time value from the second pair in the set, and continues in this
 * vein until either the required exposure time is met, or else the hardware's
 * shutter time or gain limits are reached.
 *
 * This method allows users to strike a balance between a well-exposed image and
 * an acceptable frame-rate, as opposed to simply maximising shutter time
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
 * \param[in] stages The vector of paired shutter time and gain limits
 *
 * The input stages are shutter time and _total_ gain pairs; the gain
 * encompasses both analogue and digital gain.
 *
 * The vector of stages may be empty. In that case, the helper will simply use
 * the runtime limits set through setShutterGainLimits() instead.
 */
ExposureModeHelper::ExposureModeHelper(const Span<std::pair<utils::Duration, double>> stages)
{
	minShutter_ = 0us;
	maxShutter_ = 0us;
	minGain_ = 0;
	maxGain_ = 0;

	for (const auto &[s, g] : stages) {
		shutters_.push_back(s);
		gains_.push_back(g);
	}
}

/**
 * \brief Set the shutter time and gain limits
 * \param[in] minShutter The minimum shutter time supported
 * \param[in] maxShutter The maximum shutter time supported
 * \param[in] minGain The minimum analogue gain supported
 * \param[in] maxGain The maximum analogue gain supported
 *
 * This function configures the shutter time and analogue gain limits that need
 * to be adhered to as the helper divides up exposure. Note that this function
 * *must* be called whenever those limits change and before splitExposure() is
 * used.
 *
 * If the algorithm using the helpers needs to indicate that either shutter time
 * or analogue gain or both should be fixed it can do so by setting both the
 * minima and maxima to the same value.
 */
void ExposureModeHelper::setLimits(utils::Duration minShutter,
				   utils::Duration maxShutter,
				   double minGain, double maxGain)
{
	minShutter_ = minShutter;
	maxShutter_ = maxShutter;
	minGain_ = minGain;
	maxGain_ = maxGain;
}

utils::Duration ExposureModeHelper::clampShutter(utils::Duration shutter) const
{
	return std::clamp(shutter, minShutter_, maxShutter_);
}

double ExposureModeHelper::clampGain(double gain) const
{
	return std::clamp(gain, minGain_, maxGain_);
}

/**
 * \brief Split exposure time into shutter time and gain
 * \param[in] exposure Exposure time
 *
 * This function divides a given exposure time into shutter time, analogue and
 * digital gain by iterating through stages of shutter time and gain limits. At
 * each stage the current stage's shutter time limit is multiplied by the
 * previous stage's gain limit (or 1.0 initially) to see if the combination of
 * the two can meet the required exposure time. If they cannot then the current
 * stage's shutter time limit is multiplied by the same stage's gain limit to
 * see if that combination can meet the required exposure time. If they cannot
 * then the function moves to consider the next stage.
 *
 * When a combination of shutter time and gain _stage_ limits are found that are
 * sufficient to meet the required exposure time, the function attempts to
 * reduce shutter time as much as possible whilst fixing gain and still meeting
 * the exposure time. If a _runtime_ limit prevents shutter time from being
 * lowered enough to meet the exposure time with gain fixed at the stage limit,
 * gain is also lowered to compensate.
 *
 * Once the shutter time and gain values are ascertained, gain is assigned as
 * analogue gain as much as possible, with digital gain only in use if the
 * maximum analogue gain runtime limit is unable to accommodate the exposure
 * value.
 *
 * If no combination of shutter time and gain limits is found that meets the
 * required exposure time, the helper falls-back to simply maximising the
 * shutter time first, followed by analogue gain, followed by digital gain.
 *
 * \return Tuple of shutter time, analogue gain, and digital gain
 */
std::tuple<utils::Duration, double, double>
ExposureModeHelper::splitExposure(utils::Duration exposure) const
{
	ASSERT(maxShutter_);
	ASSERT(maxGain_);

	bool gainFixed = minGain_ == maxGain_;
	bool shutterFixed = minShutter_ == maxShutter_;

	/*
	 * There's no point entering the loop if we cannot change either gain
	 * nor shutter anyway.
	 */
	if (shutterFixed && gainFixed)
		return { minShutter_, minGain_, exposure / (minShutter_ * minGain_) };

	utils::Duration shutter;
	double stageGain = 1.0;
	double gain;

	for (unsigned int stage = 0; stage < gains_.size(); stage++) {
		double lastStageGain = stage == 0 ? 1.0 : clampGain(gains_[stage - 1]);
		utils::Duration stageShutter = clampShutter(shutters_[stage]);
		stageGain = clampGain(gains_[stage]);

		/*
		 * We perform the clamping on both shutter and gain in case the
		 * helper has had limits set that prevent those values being
		 * lowered beyond a certain minimum...this can happen at runtime
		 * for various reasons and so would not be known when the stage
		 * limits are initialised.
		 */

		if (stageShutter * lastStageGain >= exposure) {
			shutter = clampShutter(exposure / clampGain(lastStageGain));
			gain = clampGain(exposure / shutter);

			return { shutter, gain, exposure / (shutter * gain) };
		}

		if (stageShutter * stageGain >= exposure) {
			shutter = clampShutter(exposure / clampGain(stageGain));
			gain = clampGain(exposure / shutter);

			return { shutter, gain, exposure / (shutter * gain) };
		}
	}

	/*
	 * From here on all we can do is max out the shutter time, followed by
	 * the analogue gain. If we still haven't achieved the target we send
	 * the rest of the exposure time to digital gain. If we were given no
	 * stages to use then the default stageGain of 1.0 is used so that
	 * shutter time is maxed before gain is touched at all.
	 */
	shutter = clampShutter(exposure / clampGain(stageGain));
	gain = clampGain(exposure / shutter);

	return { shutter, gain, exposure / (shutter * gain) };
}

/**
 * \fn ExposureModeHelper::minShutter()
 * \brief Retrieve the configured minimum shutter time limit set through
 * setShutterGainLimits()
 * \return The minShutter_ value
 */

/**
 * \fn ExposureModeHelper::maxShutter()
 * \brief Retrieve the configured maximum shutter time set through
 * setShutterGainLimits()
 * \return The maxShutter_ value
 */

/**
 * \fn ExposureModeHelper::minGain()
 * \brief Retrieve the configured minimum gain set through
 * setShutterGainLimits()
 * \return The minGain_ value
 */

/**
 * \fn ExposureModeHelper::maxGain()
 * \brief Retrieve the configured maximum gain set through
 * setShutterGainLimits()
 * \return The maxGain_ value
 */

} /* namespace ipa */

} /* namespace libcamera */
