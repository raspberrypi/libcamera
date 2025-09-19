/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * RkISP1 Wide Dynamic Range control
 */

#pragma once

#include <libcamera/control_ids.h>

#include "linux/rkisp1-config.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class WideDynamicRange : public Algorithm
{
public:
	WideDynamicRange();
	~WideDynamicRange() = default;

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
	Vector<double, 2> kneePoint(double gain, double strength);
	void applyCompensationLinear(double gain, double strength);
	void applyCompensationPower(double gain, double strength);
	void applyCompensationExponential(double gain, double strength);
	void applyHistogramEqualization(double strength);

	double exposureConstraintMaxBrightPixels_;
	double exposureConstraintY_;

	std::vector<double> hist_;

	std::array<int, RKISP1_CIF_ISP_WDR_CURVE_NUM_INTERV> toneCurveIntervalValues_;
	std::array<double, RKISP1_CIF_ISP_WDR_CURVE_NUM_INTERV + 1> toneCurveX_;
	std::array<double, RKISP1_CIF_ISP_WDR_CURVE_NUM_INTERV + 1> toneCurveY_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
