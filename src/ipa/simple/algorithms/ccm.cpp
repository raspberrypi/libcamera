/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Color correction matrix
 */

#include "ccm.h"

#include "libcamera/internal/matrix.h"

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftCcm)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Ccm::init(IPAContext &context, const ValueNode &tuningData)
{
	/* Informs the 'adjust' component that CCM is available to apply Saturation */
	context.ccmEnabled = true;

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
void Ccm::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	/* Nothing to do here, the ccm will be calculated in prepare() */
	if (frameContext.awb.autoEnabled)
		return;

	ccmAlgo_.queueRequest(context.activeState.ccm, frameContext.ccm, controls);
}

void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, [[maybe_unused]] DebayerParams *params)
{
	if (frameContext.awb.autoEnabled)
		ccmAlgo_.prepare(context.activeState.ccm, frameContext.ccm,
				 frame, frameContext.awb.colourTemperature);

	/*
	 * \todo: Split out combined matrix into individual parameters in
	 * DebayerParams and perform any pre-multiplication combination in the
	 * SoftISP component directly.
	 */
	context.activeState.combinedMatrix =
		frameContext.ccm.ccm * context.activeState.combinedMatrix;
}

void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const SwIspStats *stats,
		  ControlList &metadata)
{
	ccmAlgo_.process(frameContext.ccm, metadata);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
