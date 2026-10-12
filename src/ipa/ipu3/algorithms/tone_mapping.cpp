/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google inc.
 *
 * IPU3 ToneMapping and Gamma control
 */

#include "tone_mapping.h"

#include <cmath>
#include <span>
#include <string.h>

/**
 * \file tone_mapping.h
 */

namespace libcamera {

namespace ipa::ipu3::algorithms {

/**
 * \class ToneMapping
 * \brief A class to handle tone mapping based on gamma
 *
 * This algorithm improves the image dynamic using a look-up table which is
 * generated based on a gamma parameter.
 */

ToneMapping::ToneMapping()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int ToneMapping::init(IPAContext &context, const ValueNode &tuningData)
{
	return gammaAlgo_.init(context.ctrlMap, tuningData);
}

/**
 * \brief Configure the tone mapping given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int ToneMapping::configure(IPAContext &context,
			   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	gammaAlgo_.configure(context.activeState.gamma);
	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void ToneMapping::queueRequest(IPAContext &context, const uint32_t frame,
			       IPAFrameContext &frameContext,
			       const ControlList &controls)
{
	gammaAlgo_.queueRequest(context.activeState.gamma, frame,
				frameContext.gamma, controls);
}

/**
 * \brief Fill in the parameter structure, and enable gamma control
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The FrameContext for this frame
 * \param[out] params The IPU3 parameters
 *
 * Populate the IPU3 parameter structure with our tone mapping look up table and
 * enable the gamma control module in the processing blocks.
 */
void ToneMapping::prepare([[maybe_unused]] IPAContext &context,
			  [[maybe_unused]] const uint32_t frame,
			  IPAFrameContext &frameContext,
			  ipu3_uapi_params *params)
{
	if (!frameContext.gamma.update)
		return;

	/*
	 * Unfortunately necessary given the IPU3's gamma uAPI struct has the
	 * __packed attribute.
	 */
	uint16_t *lutData = reinterpret_cast<uint16_t *>(
		__builtin_assume_aligned(params->acc_param.gamma.gc_lut.lut, 16));
	std::span<uint16_t, kNumLutNodes> lut{ lutData, kNumLutNodes };

	gammaAlgo_.prepare(frameContext.gamma, lut);

	/* Enable the custom gamma table. */
	params->use.acc_gamma = 1;
	params->acc_param.gamma.gc_ctrl.enable = 1;
}

/**
 * \brief Calculate the tone mapping look up table
 * \param[in] context The shared IPA context
 * \param[in] frame The current frame sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The IPU3 statistics and ISP results
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * The tone mapping look up table is generated as an inverse power curve from
 * our gamma setting.
 */
void ToneMapping::process([[maybe_unused]] IPAContext &context,
			  [[maybe_unused]] const uint32_t frame,
			  IPAFrameContext &frameContext,
			  [[maybe_unused]] const ipu3_uapi_stats_3a *stats,
			  [[maybe_unused]] ControlList &metadata)
{
	gammaAlgo_.process(frameContext.gamma, metadata);
}

REGISTER_IPA_ALGORITHM(ToneMapping, "ToneMapping")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
