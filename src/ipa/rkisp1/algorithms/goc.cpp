/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 Gamma out control
 */
#include "goc.h"

#include <array>
#include <cmath>
#include <span>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/value_node.h"

#include "linux/rkisp1-config.h"

/**
 * \file goc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class GammaOutCorrection
 * \brief RkISP1 Gamma out correction
 *
 * This algorithm implements the gamma out curve for the RkISP1 using the
 * libipa GammaAlgorithm class.
 */

LOG_DEFINE_CATEGORY(RkISP1Gamma)

static constexpr unsigned int kNumLutSegments = RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10 - 1;

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int GammaOutCorrection::init(IPAContext &context, const ValueNode &tuningData)
{
	if (context.hw.numGammaOutSamples !=
	    RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10) {
		LOG(RkISP1Gamma, Error)
			<< "Gamma is not implemented for RkISP1 V12";
		return -EINVAL;
	}

	std::array<unsigned int, kNumLutSegments> segments = {
		 64,  64,  64,  64, 128, 128, 128, 128,
		256, 256, 256, 512, 512, 512, 512, 512
	};

	return gammaAlgo_.init(context.ctrlMap, tuningData, segments);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int GammaOutCorrection::configure(IPAContext &context,
				  [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	gammaAlgo_.configure(context.activeState.gamma);
	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void GammaOutCorrection::queueRequest(IPAContext &context, const uint32_t frame,
				      IPAFrameContext &frameContext,
				      const ControlList &controls)
{
	gammaAlgo_.queueRequest(context.activeState.gamma, frame,
				frameContext.gamma, controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void GammaOutCorrection::prepare(IPAContext &context,
				 [[maybe_unused]] const uint32_t frame,
				 IPAFrameContext &frameContext,
				 RkISP1Params *params)
{
	ASSERT(context.hw.numGammaOutSamples ==
	       RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10);

	if (!frameContext.gamma.update)
		return;

	auto config = params->block<BlockType::Goc>();
	config.setEnabled(true);

	std::span<uint16_t, RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10> lut{
		config->gamma_y, RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10
	};
	gammaAlgo_.prepare(frameContext.gamma, lut);
	config->mode = RKISP1_CIF_ISP_GOC_MODE_LOGARITHMIC;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void GammaOutCorrection::process([[maybe_unused]] IPAContext &context,
				 [[maybe_unused]] const uint32_t frame,
				 IPAFrameContext &frameContext,
				 [[maybe_unused]] const rkisp1_stat_buffer *stats,
				 ControlList &metadata)
{
	gammaAlgo_.process(frameContext.gamma, metadata);
}

REGISTER_IPA_ALGORITHM(GammaOutCorrection, "GammaOutCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
