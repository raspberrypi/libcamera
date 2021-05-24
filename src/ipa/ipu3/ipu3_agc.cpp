/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC control algorithm
 */

#include "ipu3_agc.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "libcamera/internal/log.h"

#include "libipa/histogram.h"

namespace libcamera {

namespace ipa::ipu3 {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait between new gain/exposure estimations */
static constexpr uint32_t kFrameSkipCount = 6;

/* Maximum ISO value for analogue gain */
static constexpr uint32_t kMinISO = 100;
static constexpr uint32_t kMaxISO = 1500;

/* Maximum analogue gain value
 * \todo grab it from a camera helper */
static constexpr uint32_t kMinGain = kMinISO / 100;
static constexpr uint32_t kMaxGain = kMaxISO / 100;

/* \todo use calculated value based on sensor */
static constexpr uint32_t kMinExposure = 1;
static constexpr uint32_t kMaxExposure = 1976;

/* \todo those should be got from IPACameraSensorInfo ! */
/* line duration in microseconds */
static constexpr double kLineDuration = 16.8;
static constexpr double kMaxExposureTime = kMaxExposure * kLineDuration;

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;
static constexpr double kEvGainTarget = 0.5;

/* A cell is 8 bytes and contains averages for RGB values and saturation ratio */
static constexpr uint8_t kCellSize = 8;

IPU3Agc::IPU3Agc()
	: frameCount_(0), lastFrame_(0), converged_(false),
	  updateControls_(false), iqMean_(0.0), gamma_(1.0),
	  prevExposure_(0.0), prevExposureNoDg_(0.0),
	  currentExposure_(0.0), currentExposureNoDg_(0.0)
{
}

void IPU3Agc::initialise(struct ipu3_uapi_grid_config &bdsGrid)
{
	aeGrid_ = bdsGrid;
}

void IPU3Agc::processBrightness(const ipu3_uapi_stats_3a *stats)
{
	const struct ipu3_uapi_grid_config statsAeGrid = stats->stats_4a_config.awb_config.grid;
	Rectangle aeRegion = { statsAeGrid.x_start,
			       statsAeGrid.y_start,
			       static_cast<unsigned int>(statsAeGrid.x_end - statsAeGrid.x_start) + 1,
			       static_cast<unsigned int>(statsAeGrid.y_end - statsAeGrid.y_start) + 1 };
	Point topleft = aeRegion.topLeft();
	int topleftX = topleft.x >> aeGrid_.block_width_log2;
	int topleftY = topleft.y >> aeGrid_.block_height_log2;

	/* Align to the grid cell width and height */
	uint32_t startX = topleftX << aeGrid_.block_width_log2;
	uint32_t startY = topleftY * aeGrid_.width << aeGrid_.block_width_log2;
	uint32_t endX = (startX + (aeRegion.size().width >> aeGrid_.block_width_log2)) << aeGrid_.block_width_log2;
	uint32_t i, j;
	uint32_t count = 0;

	uint32_t hist[knumHistogramBins] = { 0 };
	for (j = topleftY;
	     j < topleftY + (aeRegion.size().height >> aeGrid_.block_height_log2);
	     j++) {
		for (i = startX + startY; i < endX + startY; i += kCellSize) {
			/*
			 * The grid width (and maybe height) is not reliable.
			 * We observed a bit shift which makes the value 160 to be 32 in the stats grid.
			 * Use the one passed at init time.
			 */
			if (stats->awb_raw_buffer.meta_data[i + 4 + j * aeGrid_.width] == 0) {
				uint8_t Gr = stats->awb_raw_buffer.meta_data[i + 0 + j * aeGrid_.width];
				uint8_t Gb = stats->awb_raw_buffer.meta_data[i + 3 + j * aeGrid_.width];
				hist[(Gr + Gb) / 2]++;
				count++;
			}
		}
	}

	/* Limit the gamma effect for now */
	gamma_ = 1.1;

	/* Estimate the quantile mean of the top 2% of the histogram */
	iqMean_ = Histogram(Span<uint32_t>(hist)).interQuantileMean(0.98, 1.0);
}

void IPU3Agc::filterExposure()
{
	double speed = 0.2;
	if (prevExposure_ == 0.0) {
		/* DG stands for digital gain.*/
		prevExposure_ = currentExposure_;
		prevExposureNoDg_ = currentExposureNoDg_;
	} else {
		/*
		 * If we are close to the desired result, go faster to avoid making
		 * multiple micro-adjustments.
		 * \ todo: Make this customisable?
		 */
		if (prevExposure_ < 1.2 * currentExposure_ &&
		    prevExposure_ > 0.8 * currentExposure_)
			speed = sqrt(speed);

		prevExposure_ = speed * currentExposure_ +
				prevExposure_ * (1.0 - speed);
		prevExposureNoDg_ = speed * currentExposureNoDg_ +
				prevExposureNoDg_ * (1.0 - speed);
	}
	/*
	 * We can't let the no_dg exposure deviate too far below the
	 * total exposure, as there might not be enough digital gain available
	 * in the ISP to hide it (which will cause nasty oscillation).
	 */
	double fastReduceThreshold = 0.4;
	if (prevExposureNoDg_ <
	    prevExposure_ * fastReduceThreshold)
		prevExposureNoDg_ = prevExposure_ * fastReduceThreshold;
	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << prevExposure_;
}

void IPU3Agc::lockExposureGain(uint32_t &exposure, uint32_t &gain)
{
	updateControls_ = false;

	/* Algorithm initialization should wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if ((frameCount_ < kInitialFrameMinAECount) || (frameCount_ - lastFrame_ < kFrameSkipCount))
		return;

	/* Are we correctly exposed ? */
	if (std::abs(iqMean_ - kEvGainTarget * knumHistogramBins) <= 1) {
		LOG(IPU3Agc, Debug) << "!!! Good exposure with iqMean = " << iqMean_;
		converged_ = true;
	} else {
		double newGain = kEvGainTarget * knumHistogramBins / iqMean_;

		/* extracted from Rpi::Agc::computeTargetExposure */
		double currentShutter = exposure * kLineDuration;
		currentExposureNoDg_ = currentShutter * gain;
		LOG(IPU3Agc, Debug) << "Actual total exposure " << currentExposureNoDg_
				    << " Shutter speed " << currentShutter
				    << " Gain " << gain;
		currentExposure_ = currentExposureNoDg_ * newGain;
		double maxTotalExposure = kMaxExposureTime * kMaxGain;
		currentExposure_ = std::min(currentExposure_, maxTotalExposure);
		LOG(IPU3Agc, Debug) << "Target total exposure " << currentExposure_;

		/* \todo: estimate if we need to desaturate */
		filterExposure();

		double newExposure = 0.0;
		if (currentShutter < kMaxExposureTime) {
			exposure = std::clamp(static_cast<uint32_t>(exposure * currentExposure_ / currentExposureNoDg_), kMinExposure, kMaxExposure);
			newExposure = currentExposure_ / exposure;
			gain = std::clamp(static_cast<uint32_t>(gain * currentExposure_ / newExposure), kMinGain, kMaxGain);
			updateControls_ = true;
		} else if (currentShutter >= kMaxExposureTime) {
			gain = std::clamp(static_cast<uint32_t>(gain * currentExposure_ / currentExposureNoDg_), kMinGain, kMaxGain);
			newExposure = currentExposure_ / gain;
			exposure = std::clamp(static_cast<uint32_t>(exposure * currentExposure_ / newExposure), kMinExposure, kMaxExposure);
			updateControls_ = true;
		}
		LOG(IPU3Agc, Debug) << "Adjust exposure " << exposure * kLineDuration << " and gain " << gain;
	}
	lastFrame_ = frameCount_;
}

void IPU3Agc::process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain)
{
	processBrightness(stats);
	lockExposureGain(exposure, gain);
	frameCount_++;
}

} /* namespace ipa::ipu3 */

} /* namespace libcamera */
