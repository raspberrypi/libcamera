/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * IPU3 Colour correction matrix algorithm
 */

#include "ccm.h"

/**
 * \file ccm.h
 */

namespace libcamera {

namespace ipa::ipu3::algorithms {

/**
 * \class Ccm
 * \brief The IPU3 color correction matrix algorithm
 */

LOG_DEFINE_CATEGORY(IPU3Ccm)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Ccm::init(IPAContext &context, const ValueNode &tuningData)
{
	return ccmAlgo_.init(tuningData, context.ctrlMap);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Ccm::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return ccmAlgo_.configure(context.activeState.ccm,
				  context.activeState.awb.automatic.colourTemperature);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Ccm::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	/* Nothing to do here, the ccm will be calculated in prepare() */
	if (frameContext.awb.autoEnabled)
		return;

	ccmAlgo_.queueRequest(context.activeState.ccm, frameContext.ccm, controls);
}

void Ccm::setParameters(ipu3_uapi_params *params, IPAFrameContext &context)
{
	const Matrix<float, 3, 3> &matrix = context.ccm.ccm;
	const Matrix<int16_t, 3, 1> &offsets = context.ccm.offsets;

	params->use.acc_ccm = 1;

	params->acc_param.ccm.coeff_m11 = Q<3, 13>(matrix[0][0]).quantized();
	params->acc_param.ccm.coeff_m12 = Q<3, 13>(matrix[0][1]).quantized();
	params->acc_param.ccm.coeff_m13 = Q<3, 13>(matrix[0][2]).quantized();
	params->acc_param.ccm.coeff_o_r = offsets[0][0];

	params->acc_param.ccm.coeff_m21 = Q<3, 13>(matrix[1][0]).quantized();
	params->acc_param.ccm.coeff_m22 = Q<3, 13>(matrix[1][1]).quantized();
	params->acc_param.ccm.coeff_m23 = Q<3, 13>(matrix[1][2]).quantized();
	params->acc_param.ccm.coeff_o_g = offsets[1][0];

	params->acc_param.ccm.coeff_m31 = Q<3, 13>(matrix[2][0]).quantized();
	params->acc_param.ccm.coeff_m32 = Q<3, 13>(matrix[2][1]).quantized();
	params->acc_param.ccm.coeff_m33 = Q<3, 13>(matrix[2][2]).quantized();
	params->acc_param.ccm.coeff_o_b = offsets[2][0];

	LOG(IPU3Ccm, Debug) << "Setting matrix " << matrix;
	LOG(IPU3Ccm, Debug) << "Setting offsets " << offsets;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, ipu3_uapi_params *params)
{
	if (!frameContext.awb.autoEnabled) {
		setParameters(params, frameContext);
		return;
	}

	ccmAlgo_.prepare(context.activeState.ccm, frameContext.ccm, frame,
			 frameContext.awb.colourTemperature);

	setParameters(params, frameContext);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const ipu3_uapi_stats_3a *stats,
		  ControlList &metadata)
{
	ccmAlgo_.process(frameContext.ccm, metadata);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
