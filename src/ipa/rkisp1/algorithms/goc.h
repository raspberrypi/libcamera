/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 Gamma out control
 */

#pragma once

#include "algorithm.h"

#include <linux/rkisp1-config.h>

#include <libipa/fixedpoint.h>
#include <libipa/gamma.h>

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class GammaOutCorrection : public Algorithm
{
public:
	GammaOutCorrection() = default;
	~GammaOutCorrection() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context,
			  const uint32_t frame,
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
	GammaAlgorithm<RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10, UQ<0, 10>> gammaAlgo_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
