/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Black level handling
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class BlackLevel : public Algorithm
{
public:
	BlackLevel();
	~BlackLevel() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	uint32_t exposure_;
	double gain_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
