/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Ideas on Board Oy
 *
 * agc.h - Mali C55 AGC/AEC mean-based control algorithm
 */

#pragma once

#include <libcamera/base/utils.h>

#include "libcamera/internal/bayer_format.h"

#include "libipa/agc_mean_luminance.h"
#include "libipa/histogram.h"

#include "algorithm.h"
#include "ipa_context.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

class AgcStatistics
{
public:
	AgcStatistics()
	{
	}

	int setBayerOrderIndices(BayerFormat::Order bayerOrder);
	uint32_t decodeBinValue(uint16_t binVal);
	void parseStatistics(const mali_c55_stats_buffer *stats);

	Histogram rHist;
	Histogram gHist;
	Histogram bHist;
	Histogram yHist;
private:
	unsigned int rIndex_;
	unsigned int grIndex_;
	unsigned int gbIndex_;
	unsigned int bIndex_;
};

class Agc : public Algorithm, public AgcMeanLuminance
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     mali_c55_params_buffer *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mali_c55_stats_buffer *stats,
		     ControlList &metadata) override;

private:
	double estimateLuminance(const double gain) const override;
	size_t fillGainParamBlock(IPAContext &context,
				  IPAFrameContext &frameContext,
				  mali_c55_params_block block);
	size_t fillParamsBuffer(mali_c55_params_block block,
				enum mali_c55_param_block_type type);
	size_t fillWeightsArrayBuffer(mali_c55_params_block block,
				      enum mali_c55_param_block_type type);

	AgcStatistics statistics_;
};

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
