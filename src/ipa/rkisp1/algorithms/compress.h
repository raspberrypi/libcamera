/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * RkISP1 Compression curve
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Compress : public Algorithm
{
public:
	Compress() = default;
	~Compress() = default;

	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP1Params *params) override;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
