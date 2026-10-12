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

#include "libipa/agc.h"
#include "libipa/histogram.h"

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext, const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params)  override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	Histogram parseStatistics(const ipu3_uapi_stats_3a *stats,
				  const ipu3_uapi_grid_config &grid);

	uint32_t stride_;
	ipu3_uapi_grid_config bdsGrid_;
	std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> rgbTriples_;

	AgcAlgorithm agc_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
