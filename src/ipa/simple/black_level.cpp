/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * black level handling
 */

#include "black_level.h"

#include <numeric>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftBL)

/**
 * \class BlackLevel
 * \brief Object providing black point level for software ISP
 *
 * Black level can be provided in hardware tuning files or, if no tuning file is
 * available for the given hardware, guessed automatically, with less accuracy.
 * As tuning files are not yet implemented for software ISP, BlackLevel
 * currently provides only guessed black levels.
 *
 * This class serves for tracking black level as a property of the underlying
 * hardware, not as means of enhancing a particular scene or image.
 *
 * The class is supposed to be instantiated for the given camera stream.
 * The black level can be retrieved using BlackLevel::get() method. It is
 * initially 0 and may change when updated using BlackLevel::update() method.
 */

BlackLevel::BlackLevel()
	: blackLevel_(255), blackLevelSet_(false)
{
}

/**
 * \brief Return the current black level
 *
 * \return The black level, in the range from 0 (minimum) to 255 (maximum).
 * If the black level couldn't be determined yet, return 0.
 */
uint8_t BlackLevel::get() const
{
	return blackLevelSet_ ? blackLevel_ : 0;
}

/**
 * \brief Update black level from the provided histogram
 * \param[in] yHistogram The histogram to be used for updating black level
 *
 * The black level is property of the given hardware, not image. It is updated
 * only if it has not been yet set or if it is lower than the lowest value seen
 * so far.
 */
void BlackLevel::update(SwIspStats::Histogram &yHistogram)
{
	/*
	 * The constant is selected to be "good enough", not overly conservative or
	 * aggressive. There is no magic about the given value.
	 */
	constexpr float ignoredPercentage_ = 0.02;
	const unsigned int total =
		std::accumulate(begin(yHistogram), end(yHistogram), 0);
	const unsigned int pixelThreshold = ignoredPercentage_ * total;
	const unsigned int histogramRatio = 256 / SwIspStats::kYHistogramSize;
	const unsigned int currentBlackIdx = blackLevel_ / histogramRatio;

	for (unsigned int i = 0, seen = 0;
	     i < currentBlackIdx && i < SwIspStats::kYHistogramSize;
	     i++) {
		seen += yHistogram[i];
		if (seen >= pixelThreshold) {
			blackLevel_ = i * histogramRatio;
			blackLevelSet_ = true;
			LOG(IPASoftBL, Debug)
				<< "Auto-set black level: "
				<< i << "/" << SwIspStats::kYHistogramSize
				<< " (" << 100 * (seen - yHistogram[i]) / total << "% below, "
				<< 100 * seen / total << "% at or below)";
			break;
		}
	};
}
} /* namespace libcamera */
