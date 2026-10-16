/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google inc.
 *
 * IPU3 ToneMapping and Gamma control
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libipa/fixedpoint.h>
#include <libipa/gamma.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class ToneMapping : public Algorithm
{
public:
	ToneMapping();

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	static constexpr unsigned int kNumLutNodes = IPU3_UAPI_GAMMA_CORR_LUT_ENTRIES;
	GammaAlgorithm<kNumLutNodes, UQ<0, 13>> gammaAlgo_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
