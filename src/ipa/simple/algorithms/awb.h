/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat Inc.
 *
 * Auto white balance
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Awb : public Algorithm
{
public:
	Awb() = default;
	~Awb() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
