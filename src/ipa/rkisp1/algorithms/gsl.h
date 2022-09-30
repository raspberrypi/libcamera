/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * gsl.h - RkISP1 Gamma Sensor Linearization control
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class GammaSensorLinearization : public Algorithm
{
public:
	GammaSensorLinearization();
	~GammaSensorLinearization() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     rkisp1_params_cfg *params) override;

private:
	uint32_t gammaDx_[2];
	std::vector<uint16_t> curveYr_;
	std::vector<uint16_t> curveYg_;
	std::vector<uint16_t> curveYb_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
