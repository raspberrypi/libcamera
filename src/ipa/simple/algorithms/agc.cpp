/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <stdint.h>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftExposure)

namespace ipa::soft::algorithms {

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

Agc::Agc()
{
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV)
{
	/*
	 * kExpDenominator of 10 gives ~10% increment/decrement;
	 * kExpDenominator of 5 - about ~20%
	 */
	static constexpr uint8_t kExpDenominator = 10;
	static constexpr uint8_t kExpNumeratorUp = kExpDenominator + 1;
	static constexpr uint8_t kExpNumeratorDown = kExpDenominator - 1;

	double next;
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;

	if (exposureMSV < kExposureOptimal - kExposureSatisfactory) {
		next = exposure * kExpNumeratorUp / kExpDenominator;
		if (next - exposure < 1)
			exposure += 1;
		else
			exposure = next;
		if (exposure >= context.configuration.agc.exposureMax) {
			next = again * kExpNumeratorUp / kExpDenominator;
			if (next - again < context.configuration.agc.againMinStep)
				again += context.configuration.agc.againMinStep;
			else
				again = next;
		}
	}

	if (exposureMSV > kExposureOptimal + kExposureSatisfactory) {
		if (exposure == context.configuration.agc.exposureMax &&
		    again > context.configuration.agc.againMin) {
			next = again * kExpNumeratorDown / kExpDenominator;
			if (again - next < context.configuration.agc.againMinStep)
				again -= context.configuration.agc.againMinStep;
			else
				again = next;
		} else {
			next = exposure * kExpNumeratorDown / kExpDenominator;
			if (exposure - next < 1)
				exposure -= 1;
			else
				exposure = next;
		}
	}

	exposure = std::clamp(exposure, context.configuration.agc.exposureMin,
			      context.configuration.agc.exposureMax);
	again = std::clamp(again, context.configuration.agc.againMin,
			   context.configuration.agc.againMax);

	LOG(IPASoftExposure, Debug)
		<< "exposureMSV " << exposureMSV
		<< " exp " << exposure << " again " << again;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	utils::Duration exposureTime =
		context.configuration.agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const auto &histogram = stats->yHistogram;
	const unsigned int blackLevelHistIdx =
		context.activeState.blc.level / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize =
		SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	for (unsigned int i = 0; i < histogramSize; i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoftExposure, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	float exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);
	updateExposure(context, frameContext, exposureMSV);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
