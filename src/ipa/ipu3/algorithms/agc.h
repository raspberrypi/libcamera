/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * IPU3 AGC/AEC mean-based control algorithm
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "libipa/agc_mean_luminance.h"
#include "libipa/histogram.h"

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3::algorithms {

class Agc : public Algorithm, public AgcMeanLuminance
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	double estimateLuminance(double gain) const override;
	Histogram parseStatistics(const ipu3_uapi_stats_3a *stats,
				  const ipu3_uapi_grid_config &grid);

	utils::Duration minExposureTime_;
	utils::Duration maxExposureTime_;

	double minAnalogueGain_;
	double maxAnalogueGain_;

	uint32_t stride_;
	double rGain_;
	double gGain_;
	double bGain_;
	ipu3_uapi_grid_config bdsGrid_;
	std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> rgbTriples_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
