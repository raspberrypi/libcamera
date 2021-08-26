/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 *
 * This algorithm calculates a shutter time and an analogue gain so that the
 * average value of the green channel of the brightest 2% of pixels approaches
 * 0.5. The AWB gains are not used here, and all cells in the grid have the same
 * weight, like an average-metering case. In this metering mode, the camera uses
 * light information from the entire scene and creates an average for the final
 * exposure setting, giving no weighting to any particular portion of the
 * metered area.
 *
 * Reference: Battiato, Messina & Castorina. (2008). Exposure
 * Correction for Imaging Devices: An Overview. 10.1201/9781420054538.ch12.
 */

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait between new gain/shutter time estimations */
static constexpr uint32_t kFrameSkipCount = 6;

/* Limits for analogue gain values */
static constexpr double kMinAnalogueGain = 1.0;
static constexpr double kMaxAnalogueGain = 8.0;

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;

/* Target value to reach for the top 2% of the histogram */
static constexpr double kEvGainTarget = 0.5;

Agc::Agc()
	: frameCount_(0), lastFrame_(0), iqMean_(0.0), lineDuration_(0s),
	  minExposureLines_(0), maxExposureLines_(0), filteredExposure_(0s),
	  currentExposure_(0s), prevExposureValue_(0s)
{
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	stride_ = context.configuration.grid.stride;

	/* \todo use the IPAContext to provide the limits */
	lineDuration_ = configInfo.sensorInfo.lineLength * 1.0s
		      / configInfo.sensorInfo.pixelRate;

	/* \todo replace the exposure in lines storage with time based ones. */
	minExposureLines_ = context.configuration.agc.minShutterSpeed / lineDuration_;
	maxExposureLines_ = context.configuration.agc.maxShutterSpeed / lineDuration_;

	minAnalogueGain_ = std::max(context.configuration.agc.minAnalogueGain, kMinAnalogueGain);
	maxAnalogueGain_ = std::min(context.configuration.agc.maxAnalogueGain, kMaxAnalogueGain);

	/* Configure the default exposure and gain. */
	context.frameContext.agc.gain = minAnalogueGain_;
	context.frameContext.agc.exposure = minExposureLines_;

	prevExposureValue_ = context.frameContext.agc.gain
			   * context.frameContext.agc.exposure
			   * lineDuration_;

	return 0;
}

/**
 * \brief Estimate the mean value of the top 2% of the histogram
 * \param[in] stats The statistics computed by the ImgU
 * \param[in] grid The grid used to store the statistics in the IPU3
 */
void Agc::measureBrightness(const ipu3_uapi_stats_3a *stats,
			    const ipu3_uapi_grid_config &grid)
{
	/* Initialise the histogram array */
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
				/*
				 * Store the average green value to estimate the
				 * brightness. Even the overexposed pixels are
				 * taken into account.
				 */
				hist[(gr + gb) / 2]++;
			}
		}
	}

	/* Estimate the quantile mean of the top 2% of the histogram */
	iqMean_ = Histogram(Span<uint32_t>(hist)).interQuantileMean(0.98, 1.0);
}

/**
 * \brief Apply a filter on the exposure value to limit the speed of changes
 */
void Agc::filterExposure()
{
	double speed = 0.2;
	if (filteredExposure_ == 0s) {
		/* DG stands for digital gain.*/
		filteredExposure_ = currentExposure_;
	} else {
		/*
		 * If we are close to the desired result, go faster to avoid making
		 * multiple micro-adjustments.
		 * \todo Make this customisable?
		 */
		if (filteredExposure_ < 1.2 * currentExposure_ &&
		    filteredExposure_ > 0.8 * currentExposure_)
			speed = sqrt(speed);

		filteredExposure_ = speed * currentExposure_ +
				filteredExposure_ * (1.0 - speed);
	}

	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << filteredExposure_;
}

/**
 * \brief Estimate the new exposure and gain values
 * \param[inout] exposure The exposure value reference as a number of lines
 * \param[inout] gain The gain reference to be updated
 */
void Agc::computeExposure(uint32_t &exposure, double &analogueGain)
{
	/* Algorithm initialization should wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if ((frameCount_ < kInitialFrameMinAECount) || (frameCount_ - lastFrame_ < kFrameSkipCount))
		return;

	lastFrame_ = frameCount_;

	/* Are we correctly exposed ? */
	if (std::abs(iqMean_ - kEvGainTarget * knumHistogramBins) <= 1) {
		LOG(IPU3Agc, Debug) << "We are well exposed (iqMean = "
				    << iqMean_ << ")";
		return;
	}

	/* Estimate the gain needed to have the proportion wanted */
	double evGain = kEvGainTarget * knumHistogramBins / iqMean_;

	/* extracted from Rpi::Agc::computeTargetExposure */
	/* Calculate the shutter time in seconds */
	utils::Duration currentShutter = exposure * lineDuration_;
	LOG(IPU3Agc, Debug) << "Actual total exposure " << currentShutter * analogueGain
			    << " Shutter speed " << currentShutter
			    << " Gain " << analogueGain
			    << " Needed ev gain " << evGain;

	/*
	 * Calculate the current exposure value for the scene as the latest
	 * exposure value applied multiplied by the new estimated gain.
	 */
	currentExposure_ = prevExposureValue_ * evGain;
	utils::Duration minShutterSpeed = minExposureLines_ * lineDuration_;
	utils::Duration maxShutterSpeed = maxExposureLines_ * lineDuration_;

	/* Clamp the exposure value to the min and max authorized */
	utils::Duration maxTotalExposure = maxShutterSpeed * maxAnalogueGain_;
	currentExposure_ = std::min(currentExposure_, maxTotalExposure);
	LOG(IPU3Agc, Debug) << "Target total exposure " << currentExposure_
			    << ", maximum is " << maxTotalExposure;

	/* \todo: estimate if we need to desaturate */
	filterExposure();

	/* Divide the exposure value as new exposure and gain values */
	utils::Duration exposureValue = filteredExposure_;
	utils::Duration shutterTime = minShutterSpeed;

	/*
	* Push the shutter time up to the maximum first, and only then
	* increase the gain.
	*/
	shutterTime = std::clamp<utils::Duration>(exposureValue / minAnalogueGain_,
						  minShutterSpeed, maxShutterSpeed);
	double stepGain = std::clamp(exposureValue / shutterTime,
				     minAnalogueGain_, maxAnalogueGain_);
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

/**
 * \brief Process IPU3 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] stats The IPU3 statistics and ISP results
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, const ipu3_uapi_stats_3a *stats)
{
	/* Get the latest exposure and gain applied */
	uint32_t &exposure = context.frameContext.agc.exposure;
	double &analogueGain = context.frameContext.agc.gain;
	measureBrightness(stats, context.configuration.grid.bdsGrid);
	computeExposure(exposure, analogueGain);
	frameCount_++;
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
