/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.cpp - AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::rkisp1::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 */

LOG_DEFINE_CATEGORY(RkISP1Agc)

/* Limits for analogue gain values */
static constexpr double kMinAnalogueGain = 1.0;
static constexpr double kMaxAnalogueGain = 8.0;

/* \todo Honour the FrameDurationLimits control instead of hardcoding a limit */
static constexpr utils::Duration kMaxShutterSpeed = 60ms;

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kNumStartupFrames = 10;

/*
 * Relative luminance target.
 *
 * It's a number that's chosen so that, when the camera points at a grey
 * target, the resulting image brightness is considered right.
 *
 * \todo Why is the value different between IPU3 and RkISP1 ?
 */
static constexpr double kRelativeLuminanceTarget = 0.4;

Agc::Agc()
	: frameCount_(0), numCells_(0), filteredExposure_(0s)
{
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/* Configure the default exposure and gain. */
	context.frameContext.agc.gain = std::max(context.configuration.agc.minAnalogueGain, kMinAnalogueGain);
	context.frameContext.agc.exposure = 10ms / context.configuration.sensor.lineDuration;

	/*
	 * According to the RkISP1 documentation:
	 * - versions < V12 have RKISP1_CIF_ISP_AE_MEAN_MAX_V10 entries,
	 * - versions >= V12 have RKISP1_CIF_ISP_AE_MEAN_MAX_V12 entries.
	 */
	if (context.configuration.hw.revision < RKISP1_V12)
		numCells_ = RKISP1_CIF_ISP_AE_MEAN_MAX_V10;
	else
		numCells_ = RKISP1_CIF_ISP_AE_MEAN_MAX_V12;

	/* \todo Use actual frame index by populating it in the frameContext. */
	frameCount_ = 0;
	return 0;
}

/**
 * \brief Apply a filter on the exposure value to limit the speed of changes
 * \param[in] exposureValue The target exposure from the AGC algorithm
 *
 * The speed of the filter is adaptive, and will produce the target quicker
 * during startup, or when the target exposure is within 20% of the most recent
 * filter output.
 *
 * \return The filtered exposure
 */
utils::Duration Agc::filterExposure(utils::Duration exposureValue)
{
	double speed = 0.2;

	/* Adapt instantly if we are in startup phase. */
	if (frameCount_ < kNumStartupFrames)
		speed = 1.0;

	/*
	 * If we are close to the desired result, go faster to avoid making
	 * multiple micro-adjustments.
	 * \todo Make this customisable?
	 */
	if (filteredExposure_ < 1.2 * exposureValue &&
	    filteredExposure_ > 0.8 * exposureValue)
		speed = sqrt(speed);

	filteredExposure_ = speed * exposureValue +
			    filteredExposure_ * (1.0 - speed);

	LOG(RkISP1Agc, Debug) << "After filtering, exposure " << filteredExposure_;

	return filteredExposure_;
}

/**
 * \brief Estimate the new exposure and gain values
 * \param[inout] frameContext The shared IPA frame Context
 * \param[in] yGain The gain calculated on the current brightness level
 */
void Agc::computeExposure(IPAContext &context, double yGain)
{
	IPASessionConfiguration &configuration = context.configuration;
	IPAFrameContext &frameContext = context.frameContext;

	/* Get the effective exposure and gain applied on the sensor. */
	uint32_t exposure = frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;

	utils::Duration minShutterSpeed = configuration.agc.minShutterSpeed;
	utils::Duration maxShutterSpeed = std::min(configuration.agc.maxShutterSpeed,
						   kMaxShutterSpeed);

	double minAnalogueGain = std::max(configuration.agc.minAnalogueGain,
					  kMinAnalogueGain);
	double maxAnalogueGain = std::min(configuration.agc.maxAnalogueGain,
					  kMaxAnalogueGain);

	/* Consider within 1% of the target as correctly exposed. */
	if (utils::abs_diff(yGain, 1.0) < 0.01)
		return;

	/* extracted from Rpi::Agc::computeTargetExposure. */

	/* Calculate the shutter time in seconds. */
	utils::Duration currentShutter = exposure * configuration.sensor.lineDuration;

	/*
	 * Update the exposure value for the next computation using the values
	 * of exposure and gain really used by the sensor.
	 */
	utils::Duration effectiveExposureValue = currentShutter * analogueGain;

	LOG(RkISP1Agc, Debug) << "Actual total exposure " << currentShutter * analogueGain
			      << " Shutter speed " << currentShutter
			      << " Gain " << analogueGain
			      << " Needed ev gain " << yGain;

	/*
	 * Calculate the current exposure value for the scene as the latest
	 * exposure value applied multiplied by the new estimated gain.
	 */
	utils::Duration exposureValue = effectiveExposureValue * yGain;

	/* Clamp the exposure value to the min and max authorized. */
	utils::Duration maxTotalExposure = maxShutterSpeed * maxAnalogueGain;
	exposureValue = std::min(exposureValue, maxTotalExposure);
	LOG(RkISP1Agc, Debug) << "Target total exposure " << exposureValue
			      << ", maximum is " << maxTotalExposure;

	/*
	 * Divide the exposure value as new exposure and gain values.
	 * \todo estimate if we need to desaturate
	 */
	exposureValue = filterExposure(exposureValue);

	/*
	 * Push the shutter time up to the maximum first, and only then
	 * increase the gain.
	 */
	utils::Duration shutterTime = std::clamp<utils::Duration>(exposureValue / minAnalogueGain,
								  minShutterSpeed, maxShutterSpeed);
	double stepGain = std::clamp(exposureValue / shutterTime,
				     minAnalogueGain, maxAnalogueGain);
	LOG(RkISP1Agc, Debug) << "Divided up shutter and gain are "
			      << shutterTime << " and "
			      << stepGain;

	/* Update the estimated exposure and gain. */
	frameContext.agc.exposure = shutterTime / configuration.sensor.lineDuration;
	frameContext.agc.gain = stepGain;
}

/**
 * \brief Estimate the relative luminance of the frame with a given gain
 * \param[in] ae The RkISP1 statistics and ISP results
 * \param[in] gain The gain to apply to the frame
 *
 * This function estimates the average relative luminance of the frame that
 * would be output by the sensor if an additional \a gain was applied.
 *
 * The estimation is based on the AE statistics for the current frame. Y
 * averages for all cells are first multiplied by the gain, and then saturated
 * to approximate the sensor behaviour at high brightness values. The
 * approximation is quite rough, as it doesn't take into account non-linearities
 * when approaching saturation. In this case, saturating after the conversion to
 * YUV doesn't take into account the fact that the R, G and B components
 * contribute differently to the relative luminance.
 *
 * \todo Have a dedicated YUV algorithm ?
 *
 * The values are normalized to the [0.0, 1.0] range, where 1.0 corresponds to a
 * theoretical perfect reflector of 100% reference white.
 *
 * More detailed information can be found in:
 * https://en.wikipedia.org/wiki/Relative_luminance
 *
 * \return The relative luminance
 */
double Agc::estimateLuminance(const rkisp1_cif_isp_ae_stat *ae,
			      double gain)
{
	double ySum = 0.0;

	/* Sum the averages, saturated to 255. */
	for (unsigned int aeCell = 0; aeCell < numCells_; aeCell++)
		ySum += std::min(ae->exp_mean[aeCell] * gain, 255.0);

	/* \todo Weight with the AWB gains */

	return ySum / numCells_ / 255;
}

/**
 * \brief Process RkISP1 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] stats The RKISP1 statistics and ISP results
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, const rkisp1_stat_buffer *stats)
{
	const rkisp1_cif_isp_stat *params = &stats->params;
	ASSERT(stats->meas_type & RKISP1_CIF_ISP_STAT_AUTOEXP);

	const rkisp1_cif_isp_ae_stat *ae = &params->ae;

	/*
	 * Estimate the gain needed to achieve a relative luminance target. To
	 * account for non-linearity caused by saturation, the value needs to be
	 * estimated in an iterative process, as multiplying by a gain will not
	 * increase the relative luminance by the same factor if some image
	 * regions are saturated.
	 */
	double yGain = 1.0;
	double yTarget = kRelativeLuminanceTarget;

	for (unsigned int i = 0; i < 8; i++) {
		double yValue = estimateLuminance(ae, yGain);
		double extra_gain = std::min(10.0, yTarget / (yValue + .001));

		yGain *= extra_gain;
		LOG(RkISP1Agc, Debug) << "Y value: " << yValue
				      << ", Y target: " << yTarget
				      << ", gives gain " << yGain;
		if (extra_gain < 1.01)
			break;
	}

	computeExposure(context, yGain);
	frameCount_++;
}

void Agc::prepare([[maybe_unused]] IPAContext &context,
		  rkisp1_params_cfg *params)
{
	params->module_ens |= RKISP1_CIF_ISP_MODULE_AEC;
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_AEC;
}

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
