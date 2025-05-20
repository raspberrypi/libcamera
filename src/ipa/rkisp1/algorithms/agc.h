/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 AGC/AEC mean-based control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "libipa/agc_mean_luminance.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Agc : public Algorithm, public AgcMeanLuminance
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context,
			  const uint32_t frame,
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
	int parseMeteringModes(IPAContext &context, const YamlObject &tuningData);
	uint8_t computeHistogramPredivider(const Size &size,
					   enum rkisp1_cif_isp_histogram_mode mode);

	void fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
			  ControlList &metadata);
	double estimateLuminance(double gain) const override;
	void processFrameDuration(IPAContext &context,
				  IPAFrameContext &frameContext,
				  utils::Duration frameDuration);

	Span<const uint8_t> expMeans_;
	Span<const uint8_t> weights_;

	std::map<int32_t, std::vector<uint8_t>> meteringModes_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
