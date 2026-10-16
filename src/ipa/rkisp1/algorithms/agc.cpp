/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/value_node.h"

#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::rkisp1::algorithms {

LOG_DEFINE_CATEGORY(RkISP1Agc)

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 */

int Agc::parseMeteringModes(IPAContext &context, const ValueNode &tuningData)
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
			value.get<std::vector<uint8_t>>().value_or(utils::defopt);
		if (weights.size() != context.hw.numHistogramWeights) {
			LOG(RkISP1Agc, Warning)
				<< "Failed to read metering mode'" << key << "'";
			continue;
		}

		meteringModes_[controls::AeMeteringModeNameValueMap.at(key)] = weights;
	}

	if (meteringModes_.empty()) {
		LOG(RkISP1Agc, Warning)
			<< "No metering modes read from tuning file; defaulting to matrix";
		std::vector<uint8_t> weights(context.hw.numHistogramWeights, 1);

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
 * \param[in] tuningData The ValueNode containing Agc tuning data
 *
 * This function calls the base class' tuningData parsers to discover which
 * control values are supported.
 *
 * \return 0 on success or errors from the base class
 */
int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	int ret;

	ret = agc_.init(tuningData, context.camHelper.get(), {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
	if (ret)
		return ret;

	const ValueNode &meteringModes = tuningData["AeMeteringMode"];
	ret = parseMeteringModes(context, meteringModes);
	if (ret)
		return ret;

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
	int ret = agc_.configure(context.configuration.agc, context.activeState.agc, {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
		.autoAllowed = !context.configuration.raw,
	});
	if (ret)
		return ret;

	context.activeState.agc.meteringMode =
		static_cast<controls::AeMeteringModeEnum>(meteringModes_.begin()->first);

	context.configuration.agc.measureWindow.h_offs = 0;
	context.configuration.agc.measureWindow.v_offs = 0;
	context.configuration.agc.measureWindow.h_size = configInfo.outputSize.width;
	context.configuration.agc.measureWindow.v_size = configInfo.outputSize.height;

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

	agc_.queueRequest(context.configuration.agc, agc, frameContext.agc, controls);

	const auto &meteringMode = controls.get(controls::AeMeteringMode);
	if (meteringMode) {
		frameContext.agc.updateMetering = agc.meteringMode != *meteringMode;
		agc.meteringMode =
			static_cast<controls::AeMeteringModeEnum>(*meteringMode);
	}
	frameContext.agc.meteringMode = agc.meteringMode;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, RkISP1Params *params)
{
	agc_.prepare(context.activeState.agc, frameContext.agc);

	if (context.configuration.compress.supported) {
		frameContext.compress.enable = true;
		frameContext.compress.gain = frameContext.agc.quantizationGain;
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
	/*
	 * The Y mode of the histogram gets captured at the ISP output, before
	 * the output formatter.  This has the side effect that the first and
	 * the last bins are empty in case of limited YUV range.  Another side
	 * effect is that gamma and GWDR processing is included in the histogram
	 * which makes algorithm development very difficult. In RGB mode the
	 * histogram is taken after xtalk (CCM) and is therefore independent of
	 * gamma and WDR. The limited range issue also does not apply. In the
	 * ISP reference it is however stated that "it is not possible to
	 * calculate a luminance or grayscale histogram from an RGB histogram
	 * since the position information is lost during its generation".
	 *
	 * During testing the RGB histogram provided good data and better
	 * algorithmic stability at a possible (but not measured) inaccuracy.
	 *
	 * \todo For a proper fix support for HIST64 is needed.
	 */
	hstConfig->mode = RKISP1_CIF_ISP_HISTOGRAM_MODE_RGB_COMBINED;

	std::span<uint8_t> weights{
		hstConfig->hist_weight,
		context.hw.numHistogramWeights
	};
	std::vector<uint8_t> &modeWeights = meteringModes_.at(frameContext.agc.meteringMode);
	std::copy(modeWeights.begin(), modeWeights.end(), weights.begin());

	struct rkisp1_cif_isp_window window = hstConfig->meas_window;
	Size windowSize = { window.h_size, window.v_size };
	hstConfig->histogram_predivider =
		computeHistogramPredivider(windowSize,
					   static_cast<rkisp1_cif_isp_histogram_mode>(hstConfig->mode));
}

namespace {

class AgcTraits final : public AgcMeanLuminance::Traits
{
public:
	AgcTraits(std::span<const uint8_t> expMeans, std::span<const uint8_t> weights)
		: expMeans_(expMeans), weights_(weights)
	{
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
	double estimateLuminance(double gain) const override
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

private:
	std::span<const uint8_t> expMeans_;
	std::span<const uint8_t> weights_;
};

} /* namespace */

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
	/*
	 * \todo Verify that the exposure and gain applied by the sensor for
	 * this frame match what has been requested. This isn't a hard
	 * requirement for stability of the AGC (the guarantee we need in
	 * automatic mode is a perfect match between the frame and the values
	 * we receive), but is important in manual mode.
	 */

	const rkisp1_cif_isp_stat *params = nullptr;

	if (stats) {
		if (stats->meas_type & RKISP1_CIF_ISP_STAT_AUTOEXP)
			params = &stats->params;
		else
			LOG(RkISP1Agc, Error) << "AUTOEXP data is missing in statistics";
	}

	if (params) {
		std::vector<AgcMeanLuminance::AgcConstraint> additionalConstraints;
		if (context.activeState.wdr.mode != controls::WdrOff)
			additionalConstraints.push_back(context.activeState.wdr.constraint);

		agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {{
			.traits = AgcTraits{
				{ params->ae.exp_mean, context.hw.numAeCells },
				meteringModes_.at(frameContext.agc.meteringMode),
			},
			.yHist = {
				/* The lower 4 bits are fractional and meant to be discarded. */
				{ params->hist.hist_bins, context.hw.numHistogramBins },
				[](uint32_t x) { return x >> 4; },
			},
			.exposure = frameContext.sensor.exposure,
			/*
			 * Include the quantization gain if it was applied. Do not use
			 * compress.gain because it will include gains that shall not be
			 * reported to the user when HDR is implemented.
			 */
			.gain = frameContext.sensor.gain
			        * (frameContext.compress.enable ? frameContext.agc.quantizationGain : 1),
			.additionalConstraints = std::move(additionalConstraints),
			.lux = frameContext.lux.lux,
		}}, metadata);
	} else {
		agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {}, metadata);
	}

	metadata.set(controls::AeMeteringMode, frameContext.agc.meteringMode);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
