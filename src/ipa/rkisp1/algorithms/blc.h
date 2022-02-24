/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * blc.h - RkISP1 Black Level Correction control
 */

#pragma once

#include <linux/rkisp1-config.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::rkisp1::algorithms {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection() = default;
	~BlackLevelCorrection() = default;

	void prepare(IPAContext &context, rkisp1_params_cfg *params) override;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
