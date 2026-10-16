/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Mali C55 auto white balance algorithm
 */

#pragma once

#include <linux/media/arm/mali-c55-config.h>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include "libipa/awb.h"
#include "libipa/fixedpoint.h"

#include "algorithm.h"
#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

class MaliC55AwbStats;

class Awb : public Algorithm
{
public:
	~Awb() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     MaliC55Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mali_c55_stats_buffer *stats,
		     ControlList &metadata) override;

private:
	void fillConfigParamBlock(MaliC55Params *params);
	MaliC55AwbStats calculateRgbMeans(const IPAFrameContext &frameContext,
					  const mali_c55_stats_buffer *stats) const;

	AwbAlgorithm<UQ<4, 8>> awbAlgo_;
};

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
