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
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/yaml_parser.h"

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

int Agc::parseMeteringModes(IPAContext &context, const YamlObject &tuningData)
{
	if (!tuningData.isDictionary())
		LOG(RkISP1Agc, Warning)
			<< "'AeMeteringMode' parameter not found in tuning file";

	for (const auto &[key, value] : tuningData.asDict()) {
		if (controls::AeMeteringModeNameValueMap.find(key) ==
		    controls::AeMeteringModeNameValueMap.end()) {
			LOG(RkISP1Agc, Warning)
				<< "Skipping unknown metering mode '" << key << "'";
			continue;
		}

		std::vector<uint8_t> weights =
			value.getList<uint8_t>().value_or(std::vector<uint8_t>{});
		if (weights.size() != context.hw->numHistogramWeights) {
			LOG(RkISP1Agc, Warning)
				<< "Failed to read metering mode'" << key << "'";
			continue;
		}

		meteringModes_[controls::AeMeteringModeNameValueMap.at(key)] = weights;
	}

	if (meteringModes_.empty()) {
		LOG(RkISP1Agc, Warning)
			<< "No metering modes read from tuning file; defaulting to matrix";
		std::vector<uint8_t> weights(context.hw->numHistogramWeights, 1);

		meteringModes_[controls::MeteringMatrix] = weights;
	}

	std::vector<ControlValue> meteringModes;
	std::vector<int> meteringModeKeys = utils::map_keys(meteringModes_);
	std::transform(meteringModeKeys.begin(), meteringModeKeys.end(),
		       std::back_inserter(meteringModes),
		       [](int x) { return ControlValue(x); });
	context.ctrlMap[&controls::AeMeteringMode] = ControlInfo(meteringModes);

	return 0;
}

uint8_t Agc::computeHistogramPredivider(const Size &size,
					enum rkisp1_cif_isp_histogram_mode mode)
{
	/*
	 * The maximum number of pixels that could potentially be in one bin is
	 * if all the pixels of the image are in it, multiplied by 3 for the
	 * three color channels. The counter for each bin is 16 bits wide, so
	 * `factor` thus contains the number of times we'd wrap around. This is
	 * obviously the number of pixels that we need to skip to make sure
	 * that we don't wrap around, but we compute the square root of it
	 * instead, as the skip that we need to program is for both the x and y
	 * directions.
	 *
	 * Even though it looks like dividing into a counter of 65536 would
	 * overflow by 1, this is apparently fine according to the hardware
	 * documentation, and this successfully gets the expected documented
	 * predivider size for cases where:
	 * (width / predivider) * (height / predivider) * 3 == 65536.
	 *
	 * There's a bit of extra rounding math to make sure the rounding goes
	 * the correct direction so that the square of the step is big enough
	 * to encompass the `factor` number of pixels that we need to skip.
	 *
	 * \todo Take into account weights. That is, if the weights are low
	 * enough we can potentially reduce the predivider to increase
	 * precision. This needs some investigation however, as this hardware
	 * behavior is undocumented and is only an educated guess.
	 */
	int count = mode == RKISP1_CIF_ISP_HISTOGRAM_MODE_RGB_COMBINED ? 3 : 1;
	double factor = size.width * size.height * count / 65536.0;
	double root = std::sqrt(factor);
	uint8_t predivider = static_cast<uint8_t>(std::ceil(root));

	return std::clamp<uint8_t>(predivider, 3, 127);
}

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

	const YamlObject &yamlMeteringModes = tuningData["AeMeteringMode"];
	ret = parseMeteringModes(context, yamlMeteringModes);
	if (ret)
		return ret;

	context.ctrlMap[&controls::ExposureTimeMode] =
		ControlInfo({ { ControlValue(controls::ExposureTimeModeAuto),
				ControlValue(controls::ExposureTimeModeManual) } },
			    ControlValue(controls::ExposureTimeModeAuto));
	context.ctrlMap[&controls::AnalogueGainMode] =
		ControlInfo({ { ControlValue(controls::AnalogueGainModeAuto),
				ControlValue(controls::AnalogueGainModeManual) } },
			    ControlValue(controls::AnalogueGainModeAuto));
	/* \todo Move this to the Camera class */
	context.ctrlMap[&controls::AeEnable] = ControlInfo(false, true, true);
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
	context.activeState.agc.autoExposureEnabled = !context.configuration.raw;
	context.activeState.agc.autoGainEnabled = !context.configuration.raw;

	context.activeState.agc.constraintMode =
		static_cast<controls::AeConstraintModeEnum>(constraintModes().begin()->first);
	context.activeState.agc.exposureMode =
		static_cast<controls::AeExposureModeEnum>(exposureModeHelpers().begin()->first);
	context.activeState.agc.meteringMode =
		static_cast<controls::AeMeteringModeEnum>(meteringModes_.begin()->first);

	/* Limit the frame duration to match current initialisation */
	ControlInfo &frameDurationLimits = context.ctrlMap[&controls::FrameDurationLimits];
	context.activeState.agc.minFrameDuration = std::chrono::microseconds(frameDurationLimits.min().get<int64_t>());
	context.activeState.agc.maxFrameDuration = std::chrono::microseconds(frameDurationLimits.max().get<int64_t>());

	context.configuration.agc.measureWindow.h_offs = 0;
	context.configuration.agc.measureWindow.v_offs = 0;
	context.configuration.agc.measureWindow.h_size = configInfo.outputSize.width;
	context.configuration.agc.measureWindow.v_size = configInfo.outputSize.height;

	setLimits(context.configuration.sensor.minExposureTime,
		  context.configuration.sensor.maxExposureTime,
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
		const auto &aeEnable = controls.get(controls::ExposureTimeMode);
		if (aeEnable &&
		    (*aeEnable == controls::ExposureTimeModeAuto) != agc.autoExposureEnabled) {
			agc.autoExposureEnabled = (*aeEnable == controls::ExposureTimeModeAuto);

			LOG(RkISP1Agc, Debug)
				<< (agc.autoExposureEnabled ? "Enabling" : "Disabling")
				<< " AGC (exposure)";

			/*
			 * If we go from auto -> manual with no manual control
			 * set, use the last computed value, which we don't
			 * know until prepare() so save this information.
			 *
			 * \todo Check the previous frame at prepare() time
			 * instead of saving a flag here
			 */
			if (!agc.autoExposureEnabled && !controls.get(controls::ExposureTime))
				frameContext.agc.autoExposureModeChange = true;
		}

		const auto &agEnable = controls.get(controls::AnalogueGainMode);
		if (agEnable &&
		    (*agEnable == controls::AnalogueGainModeAuto) != agc.autoGainEnabled) {
			agc.autoGainEnabled = (*agEnable == controls::AnalogueGainModeAuto);

			LOG(RkISP1Agc, Debug)
				<< (agc.autoGainEnabled ? "Enabling" : "Disabling")
				<< " AGC (gain)";
			/*
			 * If we go from auto -> manual with no manual control
			 * set, use the last computed value, which we don't
			 * know until prepare() so save this information.
			 */
			if (!agc.autoGainEnabled && !controls.get(controls::AnalogueGain))
				frameContext.agc.autoGainModeChange = true;
		}
	}

	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure && !agc.autoExposureEnabled) {
		agc.manual.exposure = *exposure * 1.0us
				    / context.configuration.sensor.lineDuration;

		LOG(RkISP1Agc, Debug)
			<< "Set exposure to " << agc.manual.exposure;
	}

	const auto &gain = controls.get(controls::AnalogueGain);
	if (gain && !agc.autoGainEnabled) {
		agc.manual.gain = *gain;

		LOG(RkISP1Agc, Debug) << "Set gain to " << agc.manual.gain;
	}

	frameContext.agc.autoExposureEnabled = agc.autoExposureEnabled;
	frameContext.agc.autoGainEnabled = agc.autoGainEnabled;

	if (!frameContext.agc.autoExposureEnabled)
		frameContext.agc.exposure = agc.manual.exposure;
	if (!frameContext.agc.autoGainEnabled)
		frameContext.agc.gain = agc.manual.gain;

	const auto &meteringMode = controls.get(controls::AeMeteringMode);
	if (meteringMode) {
		frameContext.agc.updateMetering = agc.meteringMode != *meteringMode;
		agc.meteringMode =
			static_cast<controls::AeMeteringModeEnum>(*meteringMode);
	}
	frameContext.agc.meteringMode = agc.meteringMode;

	const auto &exposureMode = controls.get(controls::AeExposureMode);
	if (exposureMode)
		agc.exposureMode =
			static_cast<controls::AeExposureModeEnum>(*exposureMode);
	frameContext.agc.exposureMode = agc.exposureMode;

	const auto &constraintMode = controls.get(controls::AeConstraintMode);
	if (constraintMode)
		agc.constraintMode =
			static_cast<controls::AeConstraintModeEnum>(*constraintMode);
	frameContext.agc.constraintMode = agc.constraintMode;

	const auto &frameDurationLimits = controls.get(controls::FrameDurationLimits);
	if (frameDurationLimits) {
		/* Limit the control value to the limits in ControlInfo */
		ControlInfo &limits = context.ctrlMap[&controls::FrameDurationLimits];
		int64_t minFrameDuration =
			std::clamp((*frameDurationLimits).front(),
				   limits.min().get<int64_t>(),
				   limits.max().get<int64_t>());
		int64_t maxFrameDuration =
			std::clamp((*frameDurationLimits).back(),
				   limits.min().get<int64_t>(),
				   limits.max().get<int64_t>());

		agc.minFrameDuration = std::chrono::microseconds(minFrameDuration);
		agc.maxFrameDuration = std::chrono::microseconds(maxFrameDuration);
	}
	frameContext.agc.minFrameDuration = agc.minFrameDuration;
	frameContext.agc.maxFrameDuration = agc.maxFrameDuration;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, RkISP1Params *params)
{
	uint32_t activeAutoExposure = context.activeState.agc.automatic.exposure;
	double activeAutoGain = context.activeState.agc.automatic.gain;

	/* Populate exposure and gain in auto mode */
	if (frameContext.agc.autoExposureEnabled)
		frameContext.agc.exposure = activeAutoExposure;
	if (frameContext.agc.autoGainEnabled)
		frameContext.agc.gain = activeAutoGain;

	/*
	 * Populate manual exposure and gain from the active auto values when
	 * transitioning from auto to manual
	 */
	if (!frameContext.agc.autoExposureEnabled && frameContext.agc.autoExposureModeChange) {
		context.activeState.agc.manual.exposure = activeAutoExposure;
		frameContext.agc.exposure = activeAutoExposure;
	}
	if (!frameContext.agc.autoGainEnabled && frameContext.agc.autoGainModeChange) {
		context.activeState.agc.manual.gain = activeAutoGain;
		frameContext.agc.gain = activeAutoGain;
	}

	if (frame > 0 && !frameContext.agc.updateMetering)
		return;

	/*
	 * Configure the AEC measurements. Set the window, measure
	 * continuously, and estimate Y as (R + G + B) x (85/256).
	 */
	auto aecConfig = params->block<BlockType::Aec>();
	aecConfig.setEnabled(true);

	aecConfig->meas_window = context.configuration.agc.measureWindow;
	aecConfig->autostop = RKISP1_CIF_ISP_EXP_CTRL_AUTOSTOP_0;
	aecConfig->mode = RKISP1_CIF_ISP_EXP_MEASURING_MODE_1;

	/*
	 * Configure the histogram measurement. Set the window, produce a
	 * luminance histogram, and set the weights and predivider.
	 */
	auto hstConfig = params->block<BlockType::Hst>();
	hstConfig.setEnabled(true);

	hstConfig->meas_window = context.configuration.agc.measureWindow;
	hstConfig->mode = RKISP1_CIF_ISP_HISTOGRAM_MODE_Y_HISTOGRAM;

	Span<uint8_t> weights{
		hstConfig->hist_weight,
		context.hw->numHistogramWeights
	};
	std::vector<uint8_t> &modeWeights = meteringModes_.at(frameContext.agc.meteringMode);
	std::copy(modeWeights.begin(), modeWeights.end(), weights.begin());

	struct rkisp1_cif_isp_window window = hstConfig->meas_window;
	Size windowSize = { window.h_size, window.v_size };
	hstConfig->histogram_predivider =
		computeHistogramPredivider(windowSize,
					   static_cast<rkisp1_cif_isp_histogram_mode>(hstConfig->mode));
}

void Agc::fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
		       ControlList &metadata)
{
	utils::Duration exposureTime = context.configuration.sensor.lineDuration
				     * frameContext.sensor.exposure;
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::FrameDuration, frameContext.agc.frameDuration.get<std::micro>());
	metadata.set(controls::ExposureTimeMode,
		     frameContext.agc.autoExposureEnabled
		     ? controls::ExposureTimeModeAuto
		     : controls::ExposureTimeModeManual);
	metadata.set(controls::AnalogueGainMode,
		     frameContext.agc.autoGainEnabled
		     ? controls::AnalogueGainModeAuto
		     : controls::AnalogueGainModeManual);

	metadata.set(controls::AeMeteringMode, frameContext.agc.meteringMode);
	metadata.set(controls::AeExposureMode, frameContext.agc.exposureMode);
	metadata.set(controls::AeConstraintMode, frameContext.agc.constraintMode);
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
	ASSERT(expMeans_.size() == weights_.size());
	double ySum = 0.0;
	double wSum = 0.0;

	/* Sum the averages, saturated to 255. */
	for (unsigned i = 0; i < expMeans_.size(); i++) {
		double w = weights_[i];
		ySum += std::min(expMeans_[i] * gain, 255.0) * w;
		wSum += w;
	}

	/* \todo Weight with the AWB gains */

	return ySum / wSum / 255;
}

/**
 * \brief Process frame duration and compute vblank
 * \param[in] context The shared IPA context
 * \param[in] frameContext The current frame context
 * \param[in] frameDuration The target frame duration
 *
 * Compute and populate vblank from the target frame duration.
 */
void Agc::processFrameDuration(IPAContext &context,
			       IPAFrameContext &frameContext,
			       utils::Duration frameDuration)
{
	IPACameraSensorInfo &sensorInfo = context.sensorInfo;
	utils::Duration lineDuration = context.configuration.sensor.lineDuration;

	frameContext.agc.vblank = (frameDuration / lineDuration) - sensorInfo.outputSize.height;

	/* Update frame duration accounting for line length quantization. */
	frameContext.agc.frameDuration = (sensorInfo.outputSize.height + frameContext.agc.vblank) * lineDuration;
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
		processFrameDuration(context, frameContext,
				     frameContext.agc.minFrameDuration);
		fillMetadata(context, frameContext, metadata);
		return;
	}

	if (!(stats->meas_type & RKISP1_CIF_ISP_STAT_AUTOEXP)) {
		fillMetadata(context, frameContext, metadata);
		LOG(RkISP1Agc, Error) << "AUTOEXP data is missing in statistics";
		return;
	}

	const utils::Duration &lineDuration = context.configuration.sensor.lineDuration;

	/*
	 * \todo Verify that the exposure and gain applied by the sensor for
	 * this frame match what has been requested. This isn't a hard
	 * requirement for stability of the AGC (the guarantee we need in
	 * automatic mode is a perfect match between the frame and the values
	 * we receive), but is important in manual mode.
	 */

	const rkisp1_cif_isp_stat *params = &stats->params;

	/* The lower 4 bits are fractional and meant to be discarded. */
	Histogram hist({ params->hist.hist_bins, context.hw->numHistogramBins },
		       [](uint32_t x) { return x >> 4; });
	expMeans_ = { params->ae.exp_mean, context.hw->numAeCells };
	std::vector<uint8_t> &modeWeights = meteringModes_.at(frameContext.agc.meteringMode);
	weights_ = { modeWeights.data(), modeWeights.size() };

	/*
	 * Set the AGC limits using the fixed exposure time and/or gain in
	 * manual mode, or the sensor limits in auto mode.
	 */
	utils::Duration minExposureTime;
	utils::Duration maxExposureTime;
	double minAnalogueGain;
	double maxAnalogueGain;

	if (frameContext.agc.autoExposureEnabled) {
		minExposureTime = context.configuration.sensor.minExposureTime;
		maxExposureTime = std::clamp(frameContext.agc.maxFrameDuration,
					     context.configuration.sensor.minExposureTime,
					     context.configuration.sensor.maxExposureTime);
	} else {
		minExposureTime = context.configuration.sensor.lineDuration
				* frameContext.agc.exposure;
		maxExposureTime = minExposureTime;
	}

	if (frameContext.agc.autoGainEnabled) {
		minAnalogueGain = context.configuration.sensor.minAnalogueGain;
		maxAnalogueGain = context.configuration.sensor.maxAnalogueGain;
	} else {
		minAnalogueGain = frameContext.agc.gain;
		maxAnalogueGain = frameContext.agc.gain;
	}

	setLimits(minExposureTime, maxExposureTime, minAnalogueGain, maxAnalogueGain);

	/*
	 * The Agc algorithm needs to know the effective exposure value that was
	 * applied to the sensor when the statistics were collected.
	 */
	utils::Duration exposureTime = lineDuration * frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;
	utils::Duration effectiveExposureValue = exposureTime * analogueGain;

	utils::Duration newExposureTime;
	double aGain, dGain;
	std::tie(newExposureTime, aGain, dGain) =
		calculateNewEv(frameContext.agc.constraintMode,
			       frameContext.agc.exposureMode,
			       hist, effectiveExposureValue);

	LOG(RkISP1Agc, Debug)
		<< "Divided up exposure time, analogue gain and digital gain are "
		<< newExposureTime << ", " << aGain << " and " << dGain;

	IPAActiveState &activeState = context.activeState;
	/* Update the estimated exposure and gain. */
	activeState.agc.automatic.exposure = newExposureTime / lineDuration;
	activeState.agc.automatic.gain = aGain;

	/*
	 * Expand the target frame duration so that we do not run faster than
	 * the minimum frame duration when we have short exposures.
	 */
	processFrameDuration(context, frameContext,
			     std::max(frameContext.agc.minFrameDuration, newExposureTime));

	fillMetadata(context, frameContext, metadata);
	expMeans_ = {};
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
