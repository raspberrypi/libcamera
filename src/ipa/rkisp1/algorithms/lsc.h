/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Lens Shading Correction control
 */

#pragma once

#include <map>
#include <memory>

#include "libipa/interpolator.h"

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
		     RkISP1Params *params) override;

	struct Components {
		std::vector<uint16_t> r;
		std::vector<uint16_t> gr;
		std::vector<uint16_t> gb;
		std::vector<uint16_t> b;
	};

	class ShadingDescriptor
	{
	public:
		virtual ~ShadingDescriptor() = default;
		virtual Components sampleForCrop(const Rectangle &cropRectangle,
						 Span<const double> xSizes,
						 Span<const double> ySizes) = 0;
	};

	using ShadingDescriptorMap = std::map<unsigned int, std::unique_ptr<ShadingDescriptor>>;

private:
	void setParameters(rkisp1_cif_isp_lsc_config &config);
	void copyTable(rkisp1_cif_isp_lsc_config &config, const Components &set0);

	ShadingDescriptorMap shadingDescriptors_;
	ipa::Interpolator<Components> sets_;
	std::vector<double> xSize_;
	std::vector<double> ySize_;
	uint16_t xGrad_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	uint16_t yGrad_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	uint16_t xSizes_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];
	uint16_t ySizes_[RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE];

	unsigned int lastAppliedCt_;
	unsigned int lastAppliedQuantizedCt_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
