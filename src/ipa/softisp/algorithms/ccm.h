/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Color correction matrix
 */

#pragma once

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include "libipa/ccm.h"
#include "libipa/fixedpoint.h"

#include "algorithm.h"
#include "ipa_context.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Ccm : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;

	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	CcmAlgorithm<Q<4, 16>> ccmAlgo_;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
