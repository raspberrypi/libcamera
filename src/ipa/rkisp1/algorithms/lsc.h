/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * lsc.h - RkISP1 Lens Shading Correction control
 */

#pragma once

#include <map>

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class LensShadingCorrection : public Algorithm
{
public:
	LensShadingCorrection();
	~LensShadingCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     rkisp1_params_cfg *params) override;

private:
	struct Components {
		uint32_t ct;
		std::vector<uint16_t> r;
		std::vector<uint16_t> gr;
		std::vector<uint16_t> gb;
		std::vector<uint16_t> b;
	};

	void setParameters(rkisp1_params_cfg *params);
	void copyTable(rkisp1_cif_isp_lsc_config &config, const Components &set0);
	void interpolateTable(rkisp1_cif_isp_lsc_config &config,
			      const Components &set0, const Components &set1,
			      const uint32_t ct);

	std::map<uint32_t, Components> sets_;
	std::vector<double> xSize_;
	std::vector<double> ySize_;
	uint16_t xGrad_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	uint16_t yGrad_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	uint16_t xSizes_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	uint16_t ySizes_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	struct {
		uint32_t original;
		uint32_t adjusted;
	} lastCt_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
