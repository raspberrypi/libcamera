/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * lux.cpp - RkISP1 Lux control
 */

#include "lux.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/histogram.h"
#include "libipa/lux.h"

/**
 * \file lux.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class Lux
 * \brief RkISP1 Lux control
 *
 * The Lux algorithm is responsible for estimating the lux level of the image.
 * It doesn't take or generate any controls, but it provides a lux level for
 * other algorithms (such as AGC) to use.
 */

/**
 * \brief Construct an rkisp1 Lux algo module
 */
Lux::Lux()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Lux::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	return lux_.parseTuningData(tuningData);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Lux::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const rkisp1_stat_buffer *stats,
		  ControlList &metadata)
{
	utils::Duration exposureTime = context.configuration.sensor.lineDuration
				     * frameContext.sensor.exposure;
	double gain = frameContext.sensor.gain;

	/* \todo Deduplicate the histogram calculation from AGC */
	const rkisp1_cif_isp_stat *params = &stats->params;
	Histogram yHist({ params->hist.hist_bins, context.hw->numHistogramBins },
			[](uint32_t x) { return x >> 4; });

	double lux = lux_.estimateLux(exposureTime, gain, 1.0, yHist);
	frameContext.lux.lux = lux;
	metadata.set(controls::Lux, lux);
}

REGISTER_IPA_ALGORITHM(Lux, "Lux")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
