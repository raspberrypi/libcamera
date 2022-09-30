/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * dpcc.h - RkISP1 Defect Pixel Cluster Correction control
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class DefectPixelClusterCorrection : public Algorithm
{
public:
	DefectPixelClusterCorrection();
	~DefectPixelClusterCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     rkisp1_params_cfg *params) override;

private:
	rkisp1_cif_isp_dpcc_config config_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
