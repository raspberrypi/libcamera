/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board
 *
 * RkISP1 Compression curve
 */
#include "compress.h"

#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "linux/rkisp1-config.h"

/**
 * \file compress.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class Compress
 * \brief RkISP1 Compress curve
 *
 * This algorithm implements support for the compression curve in the compand
 * block available in the i.MX8 M Plus
 *
 * In its current version it only supports a static gain. This is useful for
 * the agc algorithm to compensate for exposure/gain quantization effects.
 *
 * This algorithm doesn't have any configuration options. It needs to be
 * configured per frame by other algorithms.
 *
 * Other algorithms can check configuration.compress.supported to see if
 * compression is available. If it is available they can configure it per frame
 * using frameContext.compress.enable and frameContext.compress.gain.
 */

LOG_DEFINE_CATEGORY(RkISP1Compress)

constexpr static int kRkISP1CompressInBits = 20;
constexpr static int kRkISP1CompressOutBits = 12;

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Compress::configure(IPAContext &context,
			[[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	if (context.configuration.paramFormat != V4L2_META_FMT_RK_ISP1_EXT_PARAMS ||
	    !context.hw.compand) {
		LOG(RkISP1Compress, Warning)
			<< "Compression is not supported by the hardware or kernel.";
		return 0;
	}

	context.configuration.compress.supported = true;
	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Compress::prepare([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       RkISP1Params *params)
{
	if (!context.configuration.compress.supported)
		return;

	auto comp = params->block<BlockType::CompandCompress>();
	comp.setEnabled(frameContext.compress.enable);

	if (!frameContext.compress.enable)
		return;

	int xmax = (1 << kRkISP1CompressInBits);
	int ymax = (1 << kRkISP1CompressOutBits);
	int inLogStep = std::log2(xmax / RKISP1_CIF_ISP_COMPAND_NUM_POINTS);

	for (unsigned int i = 0; i < RKISP1_CIF_ISP_COMPAND_NUM_POINTS; i++) {
		double x = (i + 1) * (1.0 / RKISP1_CIF_ISP_COMPAND_NUM_POINTS);
		double y = x * frameContext.compress.gain;

		comp->px[i] = inLogStep;
		comp->x[i] = std::min<int>(x * xmax, xmax - 1);
		comp->y[i] = std::min<int>(y * ymax, ymax - 1);
	}

	LOG(RkISP1Compress, Debug) << "Compression: " << kRkISP1CompressInBits
				   << " bits to " << kRkISP1CompressOutBits
				   << " bits gain: " << frameContext.compress.gain;
}

REGISTER_IPA_ALGORITHM(Compress, "Compress")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
