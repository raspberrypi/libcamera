/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 Color Correction Matrix control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include "libipa/ccm.h"
#include "libipa/fixedpoint.h"

#include "algorithm.h"
#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Ccm : public Algorithm
{
public:
	Ccm() {}
	~Ccm() = default;

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
	void setParameters(RkISP1Params *params, IPAFrameContext &context);

	CcmAlgorithm<Q<4, 7>> ccmAlgo_;
};

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
