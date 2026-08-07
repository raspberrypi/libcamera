/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat Inc.
 *
 * Auto white balance
 */

#pragma once

#include <libcamera/controls.h>

#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/value_node.h"

#include "libipa/awb.h"
#include "libipa/fixedpoint.h"
#include "simple/ipa_context.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class SimpleAwbStats;

class Awb : public Algorithm
{
public:
	Awb() = default;
	~Awb() = default;

	int init(IPAContext &context,
		 const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	SimpleAwbStats calculateRgbMeans(IPAContext &context,
					 const SwIspStats *stats) const;

	/*
	 * There actually is no Q register format for SoftISP, but allow the
	 * colour gains to range in the [0.0f, 3.999f] interval, which seems
	 * reasonable.
	 */
	AwbAlgorithm<UQ<2, 8>> awbAlgo_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
