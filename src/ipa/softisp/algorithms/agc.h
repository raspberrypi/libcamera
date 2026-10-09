/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#pragma once

#include <libipa/agc.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Agc : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext, const ControlList &controls) override;

	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, DebayerParams *params) override;

	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	AgcAlgorithm agc_;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
