/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * filter.h - RkISP1 Filter control
 */

#pragma once

#include <sys/types.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Filter : public Algorithm
{
public:
	Filter() = default;
	~Filter() = default;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     rkisp1_params_cfg *params) override;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
