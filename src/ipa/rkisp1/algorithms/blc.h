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
	BlackLevelCorrection();
	~BlackLevelCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, rkisp1_params_cfg *params) override;

private:
	bool tuningParameters_;
	int16_t blackLevelRed_;
	int16_t blackLevelGreenR_;
	int16_t blackLevelGreenB_;
	int16_t blackLevelBlue_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
