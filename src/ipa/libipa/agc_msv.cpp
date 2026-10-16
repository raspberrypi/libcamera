/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Luminance mean sample value based AGC algorithm
 */

#include "agc_msv.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include <libcamera/base/log.h>

#include "histogram.h"

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(AgcMSV)

/**
 * \class AgcMSV
 * \brief Binned mean luminance based AGC algorithm
 *
 * This is a best-effort, low-complexity auto exposure/gain algorithm based on
 * the 2007 paper "Automatic camera exposure control"
 * (https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf)
 *
 * Due to its simplicity, it is a better fit than AgcMeanLuminance when
 * operating a sensor without a known gain model.
 *
 * The algorithm operates by inspecting the luminance histogram of a frame, and
 * then splitting it into a fixed number of bins. Then an average is calculated
 * (MSV ~ mean sample value), which is compared to the target MSV. If the current
 * MSV is too low, first the exposure time, then the analogue gain is raised.
 * If it is too high, first the analogue gain, then the exposure time is
 * reduced.
 */

/**
 * \class AgcMSV::Limits
 * \brief Set of limits for the algorithm
 *
 * \var AgcMSV::Limits::exposure
 * \brief Minimum and maximum allowed exposure (in lines)
 *
 * \var AgcMSV::Limits::gain
 * \brief Minimum and maximum allowed gain
 *
 * \var AgcMSV::Limits::gainMinStep
 * \brief Minimum allowed gain adjustment
 *
 * \var AgcMSV::Limits::gain1
 * \brief The gain value assumed to result in a gain of 1.0
 *
 * The algorithm will not lower the gain value below this.
 */

/**
 * \struct AgcMSV::Params
 * \brief Collection of parameters for the algorithm
 *
 * \var AgcMSV::Params::yHist
 * \brief Luminance histogram of the frame
 *
 * \var AgcMSV::Params::exposure
 * \brief Effective exposure of the frame (in lines)
 *
 * \var AgcMSV::Params::gain
 * \brief Effective gain of the frame
 */

/**
 * \struct AgcMSV::Result
 * \brief Collection of results of the algorithm
 *
 * \var AgcMSV::Result::exposure
 * \brief The applicable exposure (in lines)
 *
 * \var AgcMSV::Result::analogueGain
 * \brief The applicable analogue gain
 */

namespace {

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;

/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range.
 */
static constexpr float kExposureOptimal = kExposureBinsCount / 2.0;

/*
 * This implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kExposureSatisfactory = 0.2;

/*
 * Proportional gain for exposure/gain adjustment. Maps the MSV error to a
 * multiplicative correction factor:
 *
 *   factor = 1.0 + kExpProportionalGain * error
 *
 * With kExpProportionalGain = 0.04:
 *   - max error ~2.5 -> factor 1.10 (~10% step, same as before)
 *   - error 1.0      -> factor 1.04 (~4% step)
 *   - error 0.3      -> factor 1.012 (~1.2% step)
 *
 * This replaces the fixed 10% bang-bang step with a proportional correction
 * that converges smoothly and avoids overshooting near the target.
 */
static constexpr float kExpProportionalGain = 0.04;

/*
 * Maximum multiplicative step per frame, to bound the correction when the
 * scene changes dramatically.
 */
static constexpr float kExpMaxStep = 0.15;

std::optional<float> calculateMSV(const Histogram &histogram)
{
	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const unsigned int yHistValsPerBin = histogram.bins() / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogram.bins() / (histogram.bins() % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	if (yHistValsPerBin == 0)
		return std::nullopt;

	for (unsigned int i = 0; i < histogram.bins(); i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(AgcMSV, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	return (denom == 0 ? 0 : static_cast<float>(num) / denom);
}

} /* namespace */

/**
 * \brief Set the limits for the algorithm
 */
void AgcMSV::setLimits(const Limits &limits)
{
	limits_ = limits;
}

/**
 * \brief Calculate a new set of AGC parameters
 */
AgcMSV::Result AgcMSV::calculateNewEv(const Params &params)
{
	AgcMSV::Result result = { params.exposure, params.gain };

	if (auto exposureMSV = calculateMSV(params.yHist)) {
		result = updateExposure(params.exposure, params.gain, *exposureMSV);
	} else {
		LOG(AgcMSV, Debug)
			<< "Not adjusting exposure due to insufficient histogram data";
	}

	result.exposure = std::clamp(result.exposure, limits_.exposure[0], limits_.exposure[1]);
	result.analogueGain = std::clamp(result.analogueGain, limits_.gain[0], limits_.gain[1]);

	LOG(AgcMSV, Debug)
		<< "exposure:" << result.exposure
		<< " analogue-gain:" << result.analogueGain;

	return result;
}

AgcMSV::Result AgcMSV::updateExposure(uint32_t exposure, double again, float exposureMSV)
{
	float error = kExposureOptimal - exposureMSV;

	LOG(AgcMSV, Debug)
		<< "exposureMSV:" << exposureMSV << " error:" << error;

	if (std::abs(error) > kExposureSatisfactory) {
		/*
		 * Compute a proportional correction factor. The sign of the error
		 * determines the direction: positive error means too dark (increase),
		 * negative means too bright (decrease).
		 */
		float step = std::clamp(error * kExpProportionalGain,
					-kExpMaxStep, kExpMaxStep);
		float factor = 1.0f + step;

		LOG(AgcMSV, Debug) << "factor:" << factor;

		if (factor > 1.0f) {
			/* Scene too dark: increase exposure first, then gain. */
			if (exposure < limits_.exposure[1]) {
				uint32_t next = exposure * factor;
				exposure = std::max(next, exposure + 1);
			} else {
				double next = again * factor;
				again = std::max(next, again + limits_.gainMinStep);
			}
		} else {
			/* Scene too bright: decrease gain first, then exposure. */
			if (again > std::max(limits_.gain1, limits_.gain[0])) {
				double next = again * factor;
				again = std::min(next, again - limits_.gainMinStep);
			} else {
				uint32_t next = exposure * factor;
				exposure = std::min(next, exposure - 1);
			}
		}
	}

	return { exposure, again };
}

} /* namespace ipa */

} /* namespace libcamera */
