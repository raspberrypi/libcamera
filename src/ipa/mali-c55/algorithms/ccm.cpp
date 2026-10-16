/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * Mali C55 Color Correction Matrix control algorithm
 */

#include "ccm.h"

#include <libcamera/base/log.h>

/**
 * \file ccm.h
 * \brief Mali-C55 CCM implementation
 */

namespace libcamera {

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Ccm)

/**
 * \class Ccm
 * \brief Mali-C55 color correction matrix algorithm
 */

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Ccm::init(IPAContext &context, const ValueNode &tuningData)
{
	auto &cmap = context.ctrlMap;
	int ret = ccmAlgo_.init(tuningData, cmap);
	if (ret)
		return ret;

	/*
	 * Mali-C55 allows to perform WB in the RGB color space as part of the
	 * CCM. Fix the gains at 1.0 as we perform White Balance in the Bayer
	 * domain.
	 */
	gain_ = 1.0;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Ccm::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	lastCt_ = context.activeState.awb.automatic.colourTemperature;
	return ccmAlgo_.configure(context.activeState.ccm, lastCt_);
}

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

void Ccm::setParameters(MaliC55Params *params, const IPAFrameContext &frameContext)
{
	auto config = params->block<MaliC55Blocks::Ccm>();

	for (unsigned int i = 0; i < 3; i++)
		config->gains[i] = UQ<4, 8>(gain_).quantized();

	for (unsigned int i = 0; i < 3; i++)
		config->offs[i] = frameContext.ccm.offsets[i][0];

	const Matrix<float, 3, 3> &ccm = frameContext.ccm.ccm;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			uint16_t val = Q<5, 8>(ccm[i][j]).quantized();

			/*
			 * CCM coefficients are expected to be in sign/magnitude
			 * format.
			 *
			 * As we're here handling the case where ccm[i][j] is
			 * negative, its quantized representation is stored in
			 * 'val' as 2's complement.
			 *
			 * We need to invert the 2's complement by
			 * re-complementing it to 1 and adding 1 back.
			 *
			 * Let's make a practical example on how to reverse the
			 * 2's complement of -7 with 4 bits and BIT(5) for sign.
			 *
			 * 2's complement (ignore sign bit)
			 *  - 1's complement of abs(-7)
			 *    2^4 - 1 - 7 = 8 -> 	1000
			 *  - Add one: 8 + 1 = 9 -> 	1001
			 *
			 * -7 is then [1 1001] in memory in 2's complement
			 *
			 * Reverse 2's complement (ignore sign bit)
			 *  - 1's complement of the 2's complement representation
			 *    2^4 - 1 - 9 = 6 ->	0110
			 *  - add one: 6 + 1 = 7 ->	0111
			 *
			 *  -7 is then [1 0111] in memory in Sign/Magnitude
			 */
			if (val && ccm[i][j] < 0)
				val = ((~(val & ~(1 << 12)) & 0x1fff) + 1) | (1 << 12);

			config->coeffs[i][j] = val;
		}
	}

	config.setEnabled(true);

	LOG(MaliC55Ccm, Debug) << "Setting ccm: " << ccm << " offsets: "
			       << frameContext.ccm.offsets
			       << " with fixed gain " << gain_;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, MaliC55Params *params)
{
	if (!frameContext.awb.autoEnabled) {
		setParameters(params, frameContext);
		return;
	}

	/*
	 * The Mali-C55 Ccm implementation is slightly stricter than the
	 * CcmAlgorithm class and only re-interpolates if the colour temperature
	 * changes of a certain amount.
	 */
	float ct = frameContext.awb.colourTemperature * 1.0f;
	if (frame > 0 && (ct < lastCt_ * 1.2 && ct > lastCt_ * 0.8)) {
		frameContext.ccm.ccm = context.activeState.ccm.automatic.ccm;
		frameContext.ccm.offsets = context.activeState.ccm.automatic.offsets;
		lastCt_ = ct;

		return;
	}

	ccmAlgo_.prepare(context.activeState.ccm, frameContext.ccm, frame, ct);
	setParameters(params, frameContext);
	lastCt_ = ct;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const mali_c55_stats_buffer *stats,
		  ControlList &metadata)
{
	ccmAlgo_.process(frameContext.ccm, metadata);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
