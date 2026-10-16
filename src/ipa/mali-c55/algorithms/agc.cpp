/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Agc)

/*
 * Number of histogram bins. This is only true for the specific configuration we
 * set to the ISP; 4 separate histograms of 256 bins each. If that configuration
 * ever changes then this constant will need updating.
 */
static constexpr unsigned int kNumHistogramBins = 256;

uint32_t AgcStatistics::decodeBinValue(uint16_t binVal)
{
	int exponent = (binVal & 0xf000) >> 12;
	int mantissa = binVal & 0xfff;

	if (!exponent)
		return mantissa * 2;
	else
		return (mantissa + 4096) * std::pow(2, exponent);
}

/*
 * We configure the ISP to give us 4 histograms of 256 bins each, with
 * a single histogram per colour channel (R/Gr/Gb/B). The memory space
 * containing the data is a single block containing all 4 histograms
 * with the position of each colour's histogram within it dependent on
 * the bayer pattern of the data input to the ISP.
 *
 * NOTE: The validity of this function depends on the parameters we have
 * configured. With different skip/offset x, y values not all of the
 * colour channels would be populated, and they may not be in the same
 * planes as calculated here.
 */
int AgcStatistics::setBayerOrderIndices(BayerFormat::Order bayerOrder)
{
	switch (bayerOrder) {
	case BayerFormat::Order::RGGB:
		rIndex_ = 0;
		grIndex_ = 1;
		gbIndex_ = 2;
		bIndex_ = 3;
		break;
	case BayerFormat::Order::GRBG:
		grIndex_ = 0;
		rIndex_ = 1;
		bIndex_ = 2;
		gbIndex_ = 3;
		break;
	case BayerFormat::Order::GBRG:
		gbIndex_ = 0;
		bIndex_ = 1;
		rIndex_ = 2;
		grIndex_ = 3;
		break;
	case BayerFormat::Order::BGGR:
		bIndex_ = 0;
		gbIndex_ = 1;
		grIndex_ = 2;
		rIndex_ = 3;
		break;
	default:
		LOG(MaliC55Agc, Error)
			<< "Invalid bayer format " << bayerOrder;
		return -EINVAL;
	}

	return 0;
}

void AgcStatistics::parseStatistics(const mali_c55_stats_buffer *stats)
{
	uint32_t r[256], g[256], b[256], y[256];

	/*
	 * We need to decode the bin values for each histogram from their 16-bit
	 * compressed values to a 32-bit value. We also take the average of the
	 * Gr/Gb values into a single green histogram.
	 */
	for (unsigned int i = 0; i < 256; i++) {
		r[i] = decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * rIndex_)]);
		g[i] = (decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * grIndex_)]) +
			decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * gbIndex_)])) / 2;
		b[i] = decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * bIndex_)]);

		y[i] = rec601LuminanceFromRGB({ { static_cast<double>(r[i]),
						  static_cast<double>(g[i]),
						  static_cast<double>(b[i]) } });
	}

	rHist = Histogram(std::span<uint32_t>(r, kNumHistogramBins));
	gHist = Histogram(std::span<uint32_t>(g, kNumHistogramBins));
	bHist = Histogram(std::span<uint32_t>(b, kNumHistogramBins));
	yHist = Histogram(std::span<uint32_t>(y, kNumHistogramBins));
}

Agc::Agc()
{
}

int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	return agc_.init(tuningData, context.camHelper.get(), {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	int ret = statistics_.setBayerOrderIndices(context.configuration.sensor.bayerOrder);
	if (ret)
		return ret;

	ret = agc_.configure(context.configuration.agc, context.activeState.agc, {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
	if (ret)
		return ret;

	return 0;
}

void Agc::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	agc_.queueRequest(context.configuration.agc, context.activeState.agc, frameContext.agc, controls);
}

void Agc::fillParamsBuffer(MaliC55Params *params, enum MaliC55Blocks type)
{
	assert(type == MaliC55Blocks::AexpHist || type == MaliC55Blocks::AexpIhist);

	auto block = type == MaliC55Blocks::AexpHist ?
			params->block<MaliC55Blocks::AexpHist>() :
			params->block<MaliC55Blocks::AexpIhist>();

	/* Collect every 3rd pixel horizontally */
	block->skip_x = 1;
	/* Start from first column */
	block->offset_x = 0;
	/* Collect every pixel vertically */
	block->skip_y = 0;
	/* Start from the first row */
	block->offset_y = 0;
	/* 1x scaling (i.e. none) */
	block->scale_bottom = 0;
	block->scale_top = 0;
	/* Collect all Bayer planes into 4 separate histograms */
	block->plane_mode = 1;
	/* Tap the data immediately after the digital gain block */
	block->tap_point = MALI_C55_AEXP_HIST_TAP_FS;
}

void Agc::fillWeightsArrayBuffer(MaliC55Params *params, const enum MaliC55Blocks type)
{
	assert(type == MaliC55Blocks::AexpHistWeights ||
	       type == MaliC55Blocks::AexpIhistWeights);

	auto block = type == MaliC55Blocks::AexpHistWeights ?
			params->block<MaliC55Blocks::AexpHistWeights>() :
			params->block<MaliC55Blocks::AexpIhistWeights>();

	/* We use every zone - a 15x15 grid */
	block->nodes_used_horiz = 15;
	block->nodes_used_vert = 15;

	/*
	 * We uniformly weight the zones to 1 - this results in the collected
	 * histograms containing a true pixel count, which we can then use to
	 * approximate colour channel averages for the image.
	 */
	std::span<uint8_t> weights{
		block->zone_weights,
		MALI_C55_MAX_ZONES
	};
	std::fill(weights.begin(), weights.end(), 1);
}

void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, MaliC55Params *params)
{
	agc_.prepare(context.activeState.agc, frameContext.agc);

	if (frame > 0)
		return;

	fillParamsBuffer(params, MaliC55Blocks::AexpHist);
	fillWeightsArrayBuffer(params, MaliC55Blocks::AexpHistWeights);

	fillParamsBuffer(params, MaliC55Blocks::AexpIhist);
	fillWeightsArrayBuffer(params, MaliC55Blocks::AexpIhistWeights);
}

namespace {

class AgcTraits final : public AgcMeanLuminance::Traits
{
public:
	AgcTraits(const AgcStatistics &statistics)
		: statistics_(statistics)
	{
	}

	double estimateLuminance(double gain) const override
	{
		double rAvg = statistics_.rHist.interQuantileMean(0, 1) * gain;
		double gAvg = statistics_.gHist.interQuantileMean(0, 1) * gain;
		double bAvg = statistics_.bHist.interQuantileMean(0, 1) * gain;
		double yAvg = rec601LuminanceFromRGB({ { rAvg, gAvg, bAvg } });

		return yAvg / kNumHistogramBins;
	}

private:
	const AgcStatistics &statistics_;
};

} /* namespace */

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const mali_c55_stats_buffer *stats,
		  ControlList &metadata)
{
	if (!stats) {
		LOG(MaliC55Agc, Error) << "No statistics buffer passed to Agc";
		return;
	}

	statistics_.parseStatistics(stats);
	context.activeState.agc.temperatureK = estimateCCT({ { statistics_.rHist.interQuantileMean(0, 1),
							       statistics_.gHist.interQuantileMean(0, 1),
							       statistics_.bHist.interQuantileMean(0, 1) } });

	agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {{
		.traits = AgcTraits(statistics_),
		.yHist = statistics_.yHist,
		.exposure = frameContext.sensor.exposure,
		.gain = frameContext.sensor.gain,
	}}, metadata);

	metadata.set(controls::ColourTemperature, context.activeState.agc.temperatureK);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
