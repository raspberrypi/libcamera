/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 Gamma out control
 */
#include "goc.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

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
 * This algorithm implements the gamma out curve for the RkISP1. It defaults to
 * a gamma value of 2.2.
 *
 * As gamma is internally represented as a piecewise linear function with only
 * 17 knots, the difference between gamma=2.2 and sRGB gamma is minimal.
 * Therefore sRGB gamma was not implemented as special case.
 *
 * Useful links:
 * - https://www.cambridgeincolour.com/tutorials/gamma-correction.htm
 * - https://en.wikipedia.org/wiki/SRGB
 */

LOG_DEFINE_CATEGORY(RkISP1Gamma)

const float kDefaultGamma = 2.2f;

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int GammaOutCorrection::init(IPAContext &context, const YamlObject &tuningData)
{
	if (context.hw->numGammaOutSamples !=
	    RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10) {
		LOG(RkISP1Gamma, Error)
			<< "Gamma is not implemented for RkISP1 V12";
		return -EINVAL;
	}

	defaultGamma_ = tuningData["gamma"].get<double>(kDefaultGamma);
	context.ctrlMap[&controls::Gamma] = ControlInfo(0.1f, 10.0f, defaultGamma_);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int GammaOutCorrection::configure(IPAContext &context,
				  [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	context.activeState.goc.gamma = defaultGamma_;
	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void GammaOutCorrection::queueRequest(IPAContext &context, const uint32_t frame,
				      IPAFrameContext &frameContext,
				      const ControlList &controls)
{
	if (frame == 0)
		frameContext.goc.update = true;

	const auto &gamma = controls.get(controls::Gamma);
	if (gamma) {
		context.activeState.goc.gamma = *gamma;
		frameContext.goc.update = true;
		LOG(RkISP1Gamma, Debug) << "Set gamma to " << *gamma;
	}

	frameContext.goc.gamma = context.activeState.goc.gamma;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void GammaOutCorrection::prepare(IPAContext &context,
				 [[maybe_unused]] const uint32_t frame,
				 IPAFrameContext &frameContext,
				 RkISP1Params *params)
{
	ASSERT(context.hw->numGammaOutSamples ==
	       RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10);

	if (!frameContext.goc.update)
		return;

	/*
	 * The logarithmic segments as specified in the reference.
	 * Plus an additional 0 to make the loop easier
	 */
	static constexpr std::array<unsigned int, RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10> segments = {
		64, 64, 64, 64, 128, 128, 128, 128, 256,
		256, 256, 512, 512, 512, 512, 512, 0
	};

	auto config = params->block<BlockType::Goc>();
	config.setEnabled(true);

	__u16 *gamma_y = config->gamma_y;

	unsigned x = 0;
	for (const auto [i, size] : utils::enumerate(segments)) {
		gamma_y[i] = std::pow(x / 4096.0, 1.0 / frameContext.goc.gamma) * 1023.0;
		x += size;
	}

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
	metadata.set(controls::Gamma, frameContext.goc.gamma);
}

REGISTER_IPA_ALGORITHM(GammaOutCorrection, "GammaOutCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
