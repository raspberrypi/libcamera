/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * IPU3 Colour correction matrix algorithm
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include <libipa/ccm.h>
#include <libipa/fixedpoint.h>

#include "algorithm.h"
#include "ipa_context.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class Ccm : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	void setParameters(ipu3_uapi_params *params, IPAFrameContext &context);
	CcmAlgorithm<Q<3, 13>> ccmAlgo_;
};

} /* namespace ipa::ipu3::algorithms */
} /* namespace libcamera */
