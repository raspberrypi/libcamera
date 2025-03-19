/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025, Red Hat Inc.
 *
 * Black level handling
 */

#include "blc.h"

#include <numeric>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftBL)

BlackLevel::BlackLevel()
{
}

int BlackLevel::init([[maybe_unused]] IPAContext &context,
		     const YamlObject &tuningData)
{
	auto blackLevel = tuningData["blackLevel"].get<int16_t>();
	if (blackLevel.has_value()) {
		/*
		 * Convert 16 bit values from the tuning file to 8 bit black
		 * level for the SoftISP.
		 */
		definedLevel_ = blackLevel.value() >> 8;
	}
	return 0;
}

int BlackLevel::configure(IPAContext &context,
			  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	if (definedLevel_.has_value())
		context.configuration.black.level = definedLevel_;
	context.activeState.blc.level =
		context.configuration.black.level.value_or(255);
	return 0;
}

void BlackLevel::process(IPAContext &context,
			 [[maybe_unused]] const uint32_t frame,
			 IPAFrameContext &frameContext,
			 const SwIspStats *stats,
			 ControlList &metadata)
{
	/* Assign each of the R G G B channels as the same black level. */
	const int32_t blackLevel = context.activeState.blc.level * 256;
	const int32_t blackLevels[] = {
		blackLevel, blackLevel, blackLevel, blackLevel
	};
	metadata.set(controls::SensorBlackLevels, blackLevels);

	if (context.configuration.black.level.has_value())
		return;

	if (frameContext.sensor.exposure == context.activeState.blc.lastExposure &&
	    frameContext.sensor.gain == context.activeState.blc.lastGain) {
		return;
	}

	const SwIspStats::Histogram &histogram = stats->yHistogram;

	/*
	 * The constant is selected to be "good enough", not overly
	 * conservative or aggressive. There is no magic about the given value.
	 */
	constexpr float ignoredPercentage = 0.02;
	const unsigned int total =
		std::accumulate(begin(histogram), end(histogram), 0);
	const unsigned int pixelThreshold = ignoredPercentage * total;
	const unsigned int histogramRatio = 256 / SwIspStats::kYHistogramSize;
	const unsigned int currentBlackIdx =
		context.activeState.blc.level / histogramRatio;

	for (unsigned int i = 0, seen = 0;
	     i < currentBlackIdx && i < SwIspStats::kYHistogramSize;
	     i++) {
		seen += histogram[i];
		if (seen >= pixelThreshold) {
			context.activeState.blc.level = i * histogramRatio;
			context.activeState.blc.lastExposure = frameContext.sensor.exposure;
			context.activeState.blc.lastGain = frameContext.sensor.gain;
			LOG(IPASoftBL, Debug)
				<< "Auto-set black level: "
				<< i << "/" << SwIspStats::kYHistogramSize
				<< " (" << 100 * (seen - histogram[i]) / total << "% below, "
				<< 100 * seen / total << "% at or below)";
			break;
		}
	};
}

REGISTER_IPA_ALGORITHM(BlackLevel, "BlackLevel")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
