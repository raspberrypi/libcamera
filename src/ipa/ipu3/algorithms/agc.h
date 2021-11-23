/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.h - IPU3 AGC/AEC mean-based control algorithm
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const ipu3_uapi_stats_3a *stats) override;

private:
	double measureBrightness(const ipu3_uapi_stats_3a *stats,
				 const ipu3_uapi_grid_config &grid) const;
	void filterExposure();
	void computeExposure(IPAFrameContext &frameContext, double yGain,
			     double iqMeanGain);
	double estimateLuminance(IPAFrameContext &frameContext,
				 const ipu3_uapi_grid_config &grid,
				 const ipu3_uapi_stats_3a *stats,
				 double gain);

	uint64_t frameCount_;

	utils::Duration lineDuration_;
	utils::Duration minShutterSpeed_;
	utils::Duration maxShutterSpeed_;

	double minAnalogueGain_;
	double maxAnalogueGain_;

	utils::Duration filteredExposure_;
	utils::Duration currentExposure_;

	uint32_t stride_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
