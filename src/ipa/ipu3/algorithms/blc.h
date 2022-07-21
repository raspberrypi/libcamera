/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google inc.
 *
 * black_correction.h - IPU3 Black Level Correction control
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection();

	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params) override;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
