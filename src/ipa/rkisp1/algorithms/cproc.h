/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * cproc.h - RkISP1 Color Processing control
 */

#pragma once

#include <sys/types.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class ColorProcessing : public Algorithm
{
public:
	ColorProcessing() = default;
	~ColorProcessing() = default;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     rkisp1_params_cfg *params) override;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
