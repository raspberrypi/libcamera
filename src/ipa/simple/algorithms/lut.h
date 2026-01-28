/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Color lookup tables construction
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Lut : public Algorithm
{
public:
	Lut() = default;
	~Lut() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;

private:
	void updateGammaTable(IPAContext &context);
	int16_t matrixValue(unsigned int i, float ccm) const;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
