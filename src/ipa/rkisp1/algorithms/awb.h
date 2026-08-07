/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * AWB control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include "libipa/awb.h"
#include "libipa/fixedpoint.h"

#include "algorithm.h"
#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class RkISP1AwbStats;

class Awb : public Algorithm
{
public:
	Awb();
	~Awb() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP1Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const rkisp1_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	RkISP1AwbStats calculateRgbMeans(const IPAFrameContext &frameContext,
					 const rkisp1_cif_isp_awb_stat *awb) const;

	AwbAlgorithm<UQ<2, 8>> awbAlgo_;

	bool rgbMode_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
