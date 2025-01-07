/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * awb.cpp - Mali C55 grey world auto white balance algorithm
 */

#include "awb.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libipa/fixedpoint.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Awb)

/* Number of frames at which we should run AWB at full speed */
static constexpr uint32_t kNumStartupFrames = 4;

Awb::Awb()
{
}

int Awb::configure([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/*
	 * Initially we have no idea what the colour balance will be like, so
	 * for the first frame we will make no assumptions and leave the R/B
	 * channels unmodified.
	 */
	context.activeState.awb.rGain = 1.0;
	context.activeState.awb.bGain = 1.0;

	return 0;
}

size_t Awb::fillGainsParamBlock(mali_c55_params_block block, IPAContext &context,
				IPAFrameContext &frameContext)
{
	block.header->type = MALI_C55_PARAM_BLOCK_AWB_GAINS;
	block.header->flags = MALI_C55_PARAM_BLOCK_FL_NONE;
	block.header->size = sizeof(struct mali_c55_params_awb_gains);

	double rGain = context.activeState.awb.rGain;
	double bGain = context.activeState.awb.bGain;

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
	block.awb_gains->gain00 = floatingToFixedPoint<4, 8, uint16_t, double>(rGain);
	block.awb_gains->gain01 = floatingToFixedPoint<4, 8, uint16_t, double>(1.0);
	block.awb_gains->gain10 = floatingToFixedPoint<4, 8, uint16_t, double>(1.0);
	block.awb_gains->gain11 = floatingToFixedPoint<4, 8, uint16_t, double>(bGain);

	frameContext.awb.rGain = rGain;
	frameContext.awb.bGain = bGain;

	return sizeof(struct mali_c55_params_awb_gains);
}

size_t Awb::fillConfigParamBlock(mali_c55_params_block block)
{
	block.header->type = MALI_C55_PARAM_BLOCK_AWB_CONFIG;
	block.header->flags = MALI_C55_PARAM_BLOCK_FL_NONE;
	block.header->size = sizeof(struct mali_c55_params_awb_config);

	/* Tap the stats after the purple fringe block */
	block.awb_config->tap_point = MALI_C55_AWB_STATS_TAP_PF;

	/* Get R/G and B/G ratios as statistics */
	block.awb_config->stats_mode = MALI_C55_AWB_MODE_RGBG;

	/* Default white level */
	block.awb_config->white_level = 1023;

	/* Default black level */
	block.awb_config->black_level = 0;

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
	block.awb_config->cr_max = 511;
	block.awb_config->cr_min = 64;
	block.awb_config->cb_max = 511;
	block.awb_config->cb_min = 64;

	/* We use the full 15x15 zoning scheme */
	block.awb_config->nodes_used_horiz = 15;
	block.awb_config->nodes_used_vert = 15;

	/*
	 * We set the trimming boundaries equivalent to the main boundaries. In
	 * other words; no trimming.
	 */
	block.awb_config->cr_high = 511;
	block.awb_config->cr_low = 64;
	block.awb_config->cb_high = 511;
	block.awb_config->cb_low = 64;

	return sizeof(struct mali_c55_params_awb_config);
}

void Awb::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, mali_c55_params_buffer *params)
{
	mali_c55_params_block block;
	block.data = &params->data[params->total_size];

	params->total_size += fillGainsParamBlock(block, context, frameContext);

	if (frame > 0)
		return;

	block.data = &params->data[params->total_size];
	params->total_size += fillConfigParamBlock(block);
}

void Awb::process(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, const mali_c55_stats_buffer *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	const struct mali_c55_awb_average_ratios *awb_ratios = stats->awb_ratios;

	/*
	 * The ISP produces average R:G and B:G ratios for zones. We take the
	 * average of all the zones with data and simply invert them to provide
	 * gain figures that we can apply to approximate a grey world.
	 */
	unsigned int counted_zones = 0;
	double rgSum = 0, bgSum = 0;

	for (unsigned int i = 0; i < 225; i++) {
		if (!awb_ratios[i].num_pixels)
			continue;

		/*
		 * The statistics are in Q4.8 format, so we convert to double
		 * here.
		 */
		rgSum += fixedToFloatingPoint<4, 8, double, uint16_t>(awb_ratios[i].avg_rg_gr);
		bgSum += fixedToFloatingPoint<4, 8, double, uint16_t>(awb_ratios[i].avg_bg_br);
		counted_zones++;
	}

	/*
	 * Sometimes the first frame's statistics have no valid pixels, in which
	 * case we'll just assume a grey world until they say otherwise.
	 */
	double rgAvg, bgAvg;
	if (!counted_zones) {
		rgAvg = 1.0;
		bgAvg = 1.0;
	} else {
		rgAvg = rgSum / counted_zones;
		bgAvg = bgSum / counted_zones;
	}

	/*
	 * The statistics are generated _after_ white balancing is performed in
	 * the ISP. To get the true ratio we therefore have to adjust the stats
	 * figure by the gains that were applied when the statistics for this
	 * frame were generated.
	 */
	double rRatio = rgAvg / frameContext.awb.rGain;
	double bRatio = bgAvg / frameContext.awb.bGain;

	/*
	 * And then we can simply invert the ratio to find the gain we should
	 * apply.
	 */
	double rGain = 1 / rRatio;
	double bGain = 1 / bRatio;

	/*
	 * Running at full speed, this algorithm results in oscillations in the
	 * colour balance. To remove those we dampen the speed at which it makes
	 * changes in gain, unless we're in the startup phase in which case we
	 * want to fix the miscolouring as quickly as possible.
	 */
	double speed = frame < kNumStartupFrames ? 1.0 : 0.2;
	rGain = speed * rGain + context.activeState.awb.rGain * (1.0 - speed);
	bGain = speed * bGain + context.activeState.awb.bGain * (1.0 - speed);

	context.activeState.awb.rGain = rGain;
	context.activeState.awb.bGain = bGain;

	metadata.set(controls::ColourGains, {
		static_cast<float>(frameContext.awb.rGain),
		static_cast<float>(frameContext.awb.bGain),
	});

	LOG(MaliC55Awb, Debug) << "For frame number " << frame << ": "
		<< "Average R/G Ratio: " << rgAvg
		<< ", Average B/G Ratio: " << bgAvg
		<< "\nrGain applied to this frame: " << frameContext.awb.rGain
		<< ", bGain applied to this frame: " << frameContext.awb.bGain
		<< "\nrGain to apply: " << context.activeState.awb.rGain
		<< ", bGain to apply: " << context.activeState.awb.bGain;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
