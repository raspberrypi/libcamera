/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * AWB control algorithm
 */

#pragma once

#include "libcamera/internal/vector.h"

#include "libipa/awb.h"
#include "libipa/interpolator.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Awb : public Algorithm
{
public:
	Awb();
	~Awb() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP1Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const rkisp1_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	RGB<double> calculateRgbMeans(const IPAFrameContext &frameContext,
				      const rkisp1_cif_isp_awb_stat *awb) const;

	std::unique_ptr<AwbAlgorithm> awbAlgo_;

	bool rgbMode_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
