/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google inc.
 *
 * tone_mapping.h - IPU3 ToneMapping and Gamma control
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class ToneMapping : public Algorithm
{
public:
	ToneMapping();

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	double gamma_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
