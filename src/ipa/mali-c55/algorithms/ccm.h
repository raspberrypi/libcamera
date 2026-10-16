/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * Mali C55 Color Correction Matrix control algorithm
 */

#pragma once

#include <linux/media/arm/mali-c55-config.h>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include "libipa/ccm.h"
#include "libipa/fixedpoint.h"

#include "algorithm.h"
#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

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
		     MaliC55Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mali_c55_stats_buffer *stats,
		     ControlList &metadata) override;

private:
	void setParameters(MaliC55Params *params, const IPAFrameContext &context);

	/*
	 * The CCM coefficient registers are said to be in Q<4,8> but this
	 * doesn't include the sign bit as the register is 13 bits wide
	 * (Q-format TI variant).
	 *
	 * As the Quantized class uses the ARM variant of the Q-format notation,
	 * make it <5, 8> to include the sign bit.
	 */
	CcmAlgorithm<Q<5, 8>> ccmAlgo_;
	float gain_;
	float lastCt_;
};

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
