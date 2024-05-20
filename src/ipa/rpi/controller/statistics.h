/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * Raspberry Pi generic statistics structure
 */
#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

#include "histogram.h"
#include "region_stats.h"

namespace RPiController {

struct RgbySums {
	RgbySums(uint64_t _rSum = 0, uint64_t _gSum = 0, uint64_t _bSum = 0, uint64_t _ySum = 0)
		: rSum(_rSum), gSum(_gSum), bSum(_bSum), ySum(_ySum)
	{
	}
	uint64_t rSum;
	uint64_t gSum;
	uint64_t bSum;
	uint64_t ySum;
};

using RgbyRegions = RegionStats<RgbySums>;
using FocusRegions = RegionStats<uint64_t>;

struct Statistics {
	/*
	 * All region based statistics are normalised to 16-bits, giving a
	 * maximum value of (1 << NormalisationFactorPow2) - 1.
	 */
	static constexpr unsigned int NormalisationFactorPow2 = 16;

	/*
	 * Positioning of the AGC statistics gathering in the pipeline:
	 * Pre-WB correction or post-WB correction.
	 * Assume this is post-LSC.
	 */
	enum class AgcStatsPos { PreWb, PostWb };
	const AgcStatsPos agcStatsPos;

	/*
	 * Positioning of the AWB/ALSC statistics gathering in the pipeline:
	 * Pre-LSC or post-LSC.
	 */
	enum class ColourStatsPos { PreLsc, PostLsc };
	const ColourStatsPos colourStatsPos;

	Statistics(AgcStatsPos a, ColourStatsPos c)
		: agcStatsPos(a), colourStatsPos(c)
	{
	}

	/* Histogram statistics. Not all histograms may be populated! */
	Histogram rHist;
	Histogram gHist;
	Histogram bHist;
	Histogram yHist;

	/* Row sums for flicker avoidance. */
	std::vector<RgbySums> rowSums;

	/* Region based colour sums. */
	RgbyRegions agcRegions;
	RgbyRegions awbRegions;

	/* Region based focus FoM. */
	FocusRegions focusRegions;
};

using StatisticsPtr = std::shared_ptr<Statistics>;

} /* namespace RPiController */
