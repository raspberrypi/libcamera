/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait between new gain/exposure estimations */
static constexpr uint32_t kFrameSkipCount = 6;

/* Maximum analogue gain value
 * \todo grab it from a camera helper */
static constexpr double kMinGain = 1.0;
static constexpr double kMaxGain = 8.0;

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;
static constexpr double kEvGainTarget = 0.5;

Agc::Agc()
	: frameCount_(0), lastFrame_(0), iqMean_(0.0), lineDuration_(0s),
	  minExposureLines_(0), maxExposureLines_(0), filteredExposure_(0s),
	  filteredExposureNoDg_(0s), currentExposure_(0s),
	  currentExposureNoDg_(0s), prevExposureValue_(0s)
{
}

int Agc::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	stride_ = context.configuration.grid.stride;

	lineDuration_ = configInfo.sensorInfo.lineLength * 1.0s
		      / configInfo.sensorInfo.pixelRate;

	/* \todo replace the exposure in lines storage with time based ones. */
	minExposureLines_ = context.configuration.agc.minShutterSpeed / lineDuration_;
	maxExposureLines_ = context.configuration.agc.maxShutterSpeed / lineDuration_;

	/* Configure the default exposure and gain. */
	context.frameContext.agc.gain =
		context.configuration.agc.minAnalogueGain;
	context.frameContext.agc.exposure = minExposureLines_;

	prevExposureValue_ = context.frameContext.agc.gain
			   * context.frameContext.agc.exposure
			   * lineDuration_;

	return 0;
}

void Agc::processBrightness(const ipu3_uapi_stats_3a *stats,
			    const ipu3_uapi_grid_config &grid)
{
	uint32_t hist[knumHistogramBins] = { 0 };

	for (unsigned int cellY = 0; cellY < grid.height; cellY++) {
		for (unsigned int cellX = 0; cellX < grid.width; cellX++) {
			uint32_t cellPosition = cellY * stride_ + cellX;

			const ipu3_uapi_awb_set_item *cell =
				reinterpret_cast<const ipu3_uapi_awb_set_item *>(
					&stats->awb_raw_buffer.meta_data[cellPosition]
				);

			if (cell->sat_ratio == 0) {
				uint8_t gr = cell->Gr_avg;
				uint8_t gb = cell->Gb_avg;
				hist[(gr + gb) / 2]++;
			}
		}
	}

	/* Estimate the quantile mean of the top 2% of the histogram */
	iqMean_ = Histogram(Span<uint32_t>(hist)).interQuantileMean(0.98, 1.0);
}

void Agc::filterExposure()
{
	double speed = 0.2;
	if (filteredExposure_ == 0s) {
		/* DG stands for digital gain.*/
		filteredExposure_ = currentExposure_;
		filteredExposureNoDg_ = currentExposureNoDg_;
	} else {
		/*
		 * If we are close to the desired result, go faster to avoid making
		 * multiple micro-adjustments.
		 * \ todo: Make this customisable?
		 */
		if (filteredExposure_ < 1.2 * currentExposure_ &&
		    filteredExposure_ > 0.8 * currentExposure_)
			speed = sqrt(speed);

		filteredExposure_ = speed * currentExposure_ +
				filteredExposure_ * (1.0 - speed);
		filteredExposureNoDg_ = speed * currentExposureNoDg_ +
				filteredExposureNoDg_ * (1.0 - speed);
	}
	/*
	 * We can't let the no_dg exposure deviate too far below the
	 * total exposure, as there might not be enough digital gain available
	 * in the ISP to hide it (which will cause nasty oscillation).
	 */
	double fastReduceThreshold = 0.4;
	if (filteredExposureNoDg_ <
	    filteredExposure_ * fastReduceThreshold)
		filteredExposureNoDg_ = filteredExposure_ * fastReduceThreshold;
	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << filteredExposure_;
}

void Agc::lockExposureGain(uint32_t &exposure, double &analogueGain)
{
	/* Algorithm initialization should wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if ((frameCount_ < kInitialFrameMinAECount) || (frameCount_ - lastFrame_ < kFrameSkipCount))
		return;

	/* Are we correctly exposed ? */
	if (std::abs(iqMean_ - kEvGainTarget * knumHistogramBins) <= 1) {
		LOG(IPU3Agc, Debug) << "!!! Good exposure with iqMean = " << iqMean_;
	} else {
		double evGain = kEvGainTarget * knumHistogramBins / iqMean_;

		/* extracted from Rpi::Agc::computeTargetExposure */
		utils::Duration currentShutter = exposure * lineDuration_;
		currentExposureNoDg_ = currentShutter * analogueGain;
		LOG(IPU3Agc, Debug) << "Actual total exposure " << currentExposureNoDg_
				    << " Shutter speed " << currentShutter
				    << " Gain " << analogueGain
				    << " Needed ev gain " << evGain;

		currentExposure_ = prevExposureValue_ * evGain;
		utils::Duration minShutterSpeed = minExposureLines_ * lineDuration_;
		utils::Duration maxShutterSpeed = maxExposureLines_ * lineDuration_;

		utils::Duration maxTotalExposure = maxShutterSpeed * kMaxGain;
		currentExposure_ = std::min(currentExposure_, maxTotalExposure);
		LOG(IPU3Agc, Debug) << "Target total exposure " << currentExposure_
				    << ", maximum is " << maxTotalExposure;

		/* \todo: estimate if we need to desaturate */
		filterExposure();

		utils::Duration exposureValue = filteredExposure_;
		utils::Duration shutterTime = minShutterSpeed;

		/*
		 * Push the shutter time up to the maximum first, and only then
		 * increase the gain.
		 */
		shutterTime = std::clamp<utils::Duration>(exposureValue / kMinGain,
							  minShutterSpeed, maxShutterSpeed);
		double stepGain = std::clamp(exposureValue / shutterTime,
					     kMinGain, kMaxGain);
		LOG(IPU3Agc, Debug) << "Divided up shutter and gain are "
				    << shutterTime << " and "
				    << stepGain;

		exposure = shutterTime / lineDuration_;
		analogueGain = stepGain;

		/*
		 * Update the exposure value for the next process call.
		 *
		 * \todo Obtain the values of the exposure time and analog gain
		 * that were actually used by the sensor, either from embedded
		 * data when available, or from the delayed controls
		 * infrastructure in case a slow down caused a mismatch.
		 */
		prevExposureValue_ = shutterTime * analogueGain;
	}
	lastFrame_ = frameCount_;
}

void Agc::process(IPAContext &context, const ipu3_uapi_stats_3a *stats)
{
	uint32_t &exposure = context.frameContext.agc.exposure;
	double &analogueGain = context.frameContext.agc.gain;
	processBrightness(stats, context.configuration.grid.bdsGrid);
	lockExposureGain(exposure, analogueGain);
	frameCount_++;
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
