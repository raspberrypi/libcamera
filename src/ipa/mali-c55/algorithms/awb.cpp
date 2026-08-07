/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * Mali C55 auto white balance algorithm
 */

#include "awb.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Awb)

/* \todo Mali-C55 doesn't support the Lux algorithm. */
static constexpr unsigned int kDefaultLux = 500;

class MaliC55AwbStats final : public AwbStats
{
public:
	MaliC55AwbStats() = default;
	MaliC55AwbStats(const RGB<double> &rgbMeans)
	{
		/* The Mali-C55 ISP already provides stats as R/G and B/G ratios. */

		rgbMeans_[0] = rgbMeans.r() * rgbMeans.g();
		rgbMeans_[1] = 1.0;
		rgbMeans_[2] = rgbMeans.b() * rgbMeans.g();

		rg_ = rgbMeans_.r();
		bg_ = rgbMeans_.b();
	}

	double computeColourError(const RGB<double> &gains) const override
	{
		/*
		 * Compute the sum of the squared colour error (non-greyness) as
		 * it appears in the log likelihood equation.
		 */
		double deltaR = gains.r() * rg_ - 1.0;
		double deltaB = gains.b() * bg_ - 1.0;
		double delta2 = deltaR * deltaR + deltaB * deltaB;

		return delta2;
	}

	RGB<double> rgbMeans() const override
	{
		return rgbMeans_;
	}

	bool valid() const override
	{
		/* Minimum mean value below which AWB can't operate. */
		constexpr double minValue = 0.2;

		return rgbMeans_.r() > minValue || rgbMeans_.g() > minValue ||
		       rgbMeans_.b() > minValue;
	}

private:
	RGB<double> rgbMeans_;
	double rg_;
	double bg_;
};

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Awb::init(IPAContext &context, const ValueNode &tuningData)
{
	return awbAlgo_.init(tuningData, context.ctrlMap);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	return awbAlgo_.configure(context.activeState.awb);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Awb::queueRequest(IPAContext &context,
		       const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	awbAlgo_.queueRequest(context.activeState.awb, frame, frameContext.awb,
			      controls);
}

void Awb::fillConfigParamBlock(MaliC55Params *params)
{
	auto block = params->block<MaliC55Blocks::AwbConfig>();

	/* Tap the stats after the purple fringe block */
	block->tap_point = MALI_C55_AWB_STATS_TAP_PF;

	/* Get R/G and B/G ratios as statistics */
	block->stats_mode = MALI_C55_AWB_MODE_RGBG;

	/* Default white level */
	block->white_level = 1023;

	/* Default black level */
	block->black_level = 0;

	/*
	 * By default pixels are included who's colour ratios are bounded in a
	 * region (on a cr ratio x cb ratio graph) defined by four points:
	 *	(0.25, 0.25)
	 *	(0.25, 1.99609375)
	 *	(1.99609375, 1.99609375)
	 *	(1.99609375, 0.25)
	 *
	 * The ratios themselves are stored in Q4.8 format.
	 *
	 * \todo should these perhaps be tunable?
	 */
	block->cr_max = 511;
	block->cr_min = 64;
	block->cb_max = 511;
	block->cb_min = 64;

	/* We use the full 15x15 zoning scheme */
	block->nodes_used_horiz = 15;
	block->nodes_used_vert = 15;

	/*
	 * We set the trimming boundaries equivalent to the main boundaries. In
	 * other words; no trimming.
	 */
	block->cr_high = 511;
	block->cr_low = 64;
	block->cb_high = 511;
	block->cb_low = 64;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, MaliC55Params *params)
{
	awbAlgo_.prepare(context.activeState.awb, frameContext.awb);

	/*
	 * The gains here map as follows:
	 *	gain00 = R
	 *	gain01 = Gr
	 *	gain10 = Gb
	 *	gain11 = B
	 *
	 * This holds true regardless of the bayer order of the input data, as
	 * the mapping is done internally in the ISP.
	 */
	auto block = params->block<MaliC55Blocks::AwbGains>();
	block.setEnabled(true);

	block->gain00 = UQ<4, 8>(static_cast<float>(frameContext.awb.gains.r()))
				.quantized();
	block->gain01 = UQ<4, 8>(1.0f).quantized();
	block->gain10 = UQ<4, 8>(1.0f).quantized();
	block->gain11 = UQ<4, 8>(static_cast<float>(frameContext.awb.gains.b()))
				.quantized();

	if (frame > 0)
		return;

	fillConfigParamBlock(params);
}

MaliC55AwbStats Awb::calculateRgbMeans(const IPAFrameContext &frameContext,
				       const mali_c55_stats_buffer *stats) const
{
	const struct mali_c55_awb_average_ratios *awb = stats->awb_ratios;

	/*
	 * The ISP produces average R:G and B:G ratios for zones. We take the
	 * average of all the zones with data and calculate the mean values.
	 */
	unsigned int active_zones = 0;
	double rgSum = 0, bgSum = 0;

	for (unsigned int i = 0; i < 225; i++) {
		if (!awb[i].num_pixels)
			continue;

		/*
		 * The statistics are in Q4.8 format, so we convert to double
		 * here.
		 */
		rgSum += UQ<4, 8>(awb[i].avg_rg_gr).value();
		bgSum += UQ<4, 8>(awb[i].avg_bg_br).value();
		active_zones++;
	}

	/*
	 * Sometimes the first frame's statistics have no valid pixels, in which
	 * case we'll just assume a grey world until they say otherwise.
	 */
	if (!active_zones)
		return {};

	RGB<double> rgbMeans = { {
		rgSum / active_zones,
		1.0,
		bgSum / active_zones,
	} };

	/*
	 * The statistics are generated _after_ white balancing is performed in
	 * the ISP. To get the true ratio we therefore have to adjust the stats
	 * figure by the gains that were applied when the statistics for this
	 * frame were generated.
	 */
	rgbMeans /= frameContext.awb.gains.max(0.01);

	return MaliC55AwbStats(rgbMeans);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Awb::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, const mali_c55_stats_buffer *stats,
		  ControlList &metadata)
{
	MaliC55AwbStats awbStats = calculateRgbMeans(frameContext, stats);

	awbAlgo_.process(context.activeState.awb, frameContext.awb, awbStats,
			 kDefaultLux, metadata);
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
