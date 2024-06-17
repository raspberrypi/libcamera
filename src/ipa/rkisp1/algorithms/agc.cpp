/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

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

Agc::Agc()
{
	supportsRaw_ = true;
}

/**
 * \brief Initialise the AGC algorithm from tuning files
 * \param[in] context The shared IPA context
 * \param[in] tuningData The YamlObject containing Agc tuning data
 *
 * This function calls the base class' tuningData parsers to discover which
 * control values are supported.
 *
 * \return 0 on success or errors from the base class
 */
int Agc::init(IPAContext &context, const YamlObject &tuningData)
{
	int ret;

	ret = parseTuningData(tuningData);
	if (ret)
		return ret;

	context.ctrlMap.merge(controls());

	return 0;
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	/* Configure the default exposure and gain. */
	context.activeState.agc.automatic.gain = context.configuration.sensor.minAnalogueGain;
	context.activeState.agc.automatic.exposure =
		10ms / context.configuration.sensor.lineDuration;
	context.activeState.agc.manual.gain = context.activeState.agc.automatic.gain;
	context.activeState.agc.manual.exposure = context.activeState.agc.automatic.exposure;
	context.activeState.agc.autoEnabled = !context.configuration.raw;

	context.activeState.agc.constraintMode = constraintModes().begin()->first;
	context.activeState.agc.exposureMode = exposureModeHelpers().begin()->first;

	/*
	 * Define the measurement window for AGC as a centered rectangle
	 * covering 3/4 of the image width and height.
	 */
	context.configuration.agc.measureWindow.h_offs = configInfo.outputSize.width / 8;
	context.configuration.agc.measureWindow.v_offs = configInfo.outputSize.height / 8;
	context.configuration.agc.measureWindow.h_size = 3 * configInfo.outputSize.width / 4;
	context.configuration.agc.measureWindow.v_size = 3 * configInfo.outputSize.height / 4;

	/* \todo Run this again when FrameDurationLimits is passed in */
	setLimits(context.configuration.sensor.minShutterSpeed,
		  context.configuration.sensor.maxShutterSpeed,
		  context.configuration.sensor.minAnalogueGain,
		  context.configuration.sensor.maxAnalogueGain);

	resetFrameCount();

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &agc = context.activeState.agc;

	if (!context.configuration.raw) {
		const auto &agcEnable = controls.get(controls::AeEnable);
		if (agcEnable && *agcEnable != agc.autoEnabled) {
			agc.autoEnabled = *agcEnable;

			LOG(RkISP1Agc, Debug)
				<< (agc.autoEnabled ? "Enabling" : "Disabling")
				<< " AGC";
		}
	}

	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure && !agc.autoEnabled) {
		agc.manual.exposure = *exposure * 1.0us
				    / context.configuration.sensor.lineDuration;

		LOG(RkISP1Agc, Debug)
			<< "Set exposure to " << agc.manual.exposure;
	}

	const auto &gain = controls.get(controls::AnalogueGain);
	if (gain && !agc.autoEnabled) {
		agc.manual.gain = *gain;

		LOG(RkISP1Agc, Debug) << "Set gain to " << agc.manual.gain;
	}

	frameContext.agc.autoEnabled = agc.autoEnabled;

	if (!frameContext.agc.autoEnabled) {
		frameContext.agc.exposure = agc.manual.exposure;
		frameContext.agc.gain = agc.manual.gain;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, rkisp1_params_cfg *params)
{
	if (frameContext.agc.autoEnabled) {
		frameContext.agc.exposure = context.activeState.agc.automatic.exposure;
		frameContext.agc.gain = context.activeState.agc.automatic.gain;
	}

	if (frame > 0)
		return;

	/* Configure the measurement window. */
	params->meas.aec_config.meas_window = context.configuration.agc.measureWindow;
	/* Use a continuous method for measure. */
	params->meas.aec_config.autostop = RKISP1_CIF_ISP_EXP_CTRL_AUTOSTOP_0;
	/* Estimate Y as (R + G + B) x (85/256). */
	params->meas.aec_config.mode = RKISP1_CIF_ISP_EXP_MEASURING_MODE_1;

	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_AEC;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_AEC;
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_AEC;

	/* Configure histogram. */
	params->meas.hst_config.meas_window = context.configuration.agc.measureWindow;
	/* Produce the luminance histogram. */
	params->meas.hst_config.mode = RKISP1_CIF_ISP_HISTOGRAM_MODE_Y_HISTOGRAM;
	/* Set an average weighted histogram. */
	Span<uint8_t> weights{
		params->meas.hst_config.hist_weight,
		context.hw->numHistogramWeights
	};
	std::fill(weights.begin(), weights.end(), 1);
	/* Step size can't be less than 3. */
	params->meas.hst_config.histogram_predivider = 4;

	/* Update the configuration for histogram. */
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_HST;
	/* Enable the histogram measure unit. */
	params->module_ens |= RKISP1_CIF_ISP_MODULE_HST;
	params->module_en_update |= RKISP1_CIF_ISP_MODULE_HST;
}

void Agc::fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
		       ControlList &metadata)
{
	utils::Duration exposureTime = context.configuration.sensor.lineDuration
				     * frameContext.sensor.exposure;
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());

	/* \todo Use VBlank value calculated from each frame exposure. */
	uint32_t vTotal = context.configuration.sensor.size.height
			+ context.configuration.sensor.defVBlank;
	utils::Duration frameDuration = context.configuration.sensor.lineDuration
				      * vTotal;
	metadata.set(controls::FrameDuration, frameDuration.get<std::micro>());
}

/**
 * \brief Estimate the relative luminance of the frame with a given gain
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
 * The values are normalized to the [0.0, 1.0] range, where 1.0 corresponds to a
 * theoretical perfect reflector of 100% reference white.
 *
 * More detailed information can be found in:
 * https://en.wikipedia.org/wiki/Relative_luminance
 *
 * \return The relative luminance
 */
double Agc::estimateLuminance(double gain) const
{
	double ySum = 0.0;

	/* Sum the averages, saturated to 255. */
	for (uint8_t expMean : expMeans_)
		ySum += std::min(expMean * gain, 255.0);

	/* \todo Weight with the AWB gains */

	return ySum / expMeans_.size() / 255;
}

/**
 * \brief Process RkISP1 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The RKISP1 statistics and ISP results
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, const rkisp1_stat_buffer *stats,
		  ControlList &metadata)
{
	if (!stats) {
		fillMetadata(context, frameContext, metadata);
		return;
	}

	/*
	 * \todo Verify that the exposure and gain applied by the sensor for
	 * this frame match what has been requested. This isn't a hard
	 * requirement for stability of the AGC (the guarantee we need in
	 * automatic mode is a perfect match between the frame and the values
	 * we receive), but is important in manual mode.
	 */

	const rkisp1_cif_isp_stat *params = &stats->params;
	ASSERT(stats->meas_type & RKISP1_CIF_ISP_STAT_AUTOEXP);

	/* The lower 4 bits are fractional and meant to be discarded. */
	Histogram hist({ params->hist.hist_bins, context.hw->numHistogramBins },
		       [](uint32_t x) { return x >> 4; });
	expMeans_ = { params->ae.exp_mean, context.hw->numAeCells };

	/*
	 * The Agc algorithm needs to know the effective exposure value that was
	 * applied to the sensor when the statistics were collected.
	 */
	utils::Duration exposureTime = context.configuration.sensor.lineDuration
				       * frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;
	utils::Duration effectiveExposureValue = exposureTime * analogueGain;

	utils::Duration shutterTime;
	double aGain, dGain;
	std::tie(shutterTime, aGain, dGain) =
		calculateNewEv(context.activeState.agc.constraintMode,
			       context.activeState.agc.exposureMode,
			       hist, effectiveExposureValue);

	LOG(RkISP1Agc, Debug)
		<< "Divided up shutter, analogue gain and digital gain are "
		<< shutterTime << ", " << aGain << " and " << dGain;

	IPAActiveState &activeState = context.activeState;
	/* Update the estimated exposure and gain. */
	activeState.agc.automatic.exposure = shutterTime / context.configuration.sensor.lineDuration;
	activeState.agc.automatic.gain = aGain;

	fillMetadata(context, frameContext, metadata);
	expMeans_ = {};
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
