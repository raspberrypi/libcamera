/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * agc.cpp - AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>

#include "libipa/colours.h"
#include "libipa/fixedpoint.h"

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::mali_c55::algorithms {

LOG_DEFINE_CATEGORY(MaliC55Agc)

/*
 * Number of histogram bins. This is only true for the specific configuration we
 * set to the ISP; 4 separate histograms of 256 bins each. If that configuration
 * ever changes then this constant will need updating.
 */
static constexpr unsigned int kNumHistogramBins = 256;

/*
 * The Mali-C55 ISP has a digital gain block which allows setting gain in Q5.8
 * format, a range of 0.0 to (very nearly) 32.0. We clamp from 1.0 to the actual
 * max value which is 8191 * 2^-8.
 */
static constexpr double kMinDigitalGain = 1.0;
static constexpr double kMaxDigitalGain = 31.99609375;

uint32_t AgcStatistics::decodeBinValue(uint16_t binVal)
{
	int exponent = (binVal & 0xf000) >> 12;
	int mantissa = binVal & 0xfff;

	if (!exponent)
		return mantissa * 2;
	else
		return (mantissa + 4096) * std::pow(2, exponent);
}

/*
 * We configure the ISP to give us 4 histograms of 256 bins each, with
 * a single histogram per colour channel (R/Gr/Gb/B). The memory space
 * containing the data is a single block containing all 4 histograms
 * with the position of each colour's histogram within it dependent on
 * the bayer pattern of the data input to the ISP.
 *
 * NOTE: The validity of this function depends on the parameters we have
 * configured. With different skip/offset x, y values not all of the
 * colour channels would be populated, and they may not be in the same
 * planes as calculated here.
 */
int AgcStatistics::setBayerOrderIndices(BayerFormat::Order bayerOrder)
{
	switch (bayerOrder) {
	case BayerFormat::Order::RGGB:
		rIndex_ = 0;
		grIndex_ = 1;
		gbIndex_ = 2;
		bIndex_ = 3;
		break;
	case BayerFormat::Order::GRBG:
		grIndex_ = 0;
		rIndex_ = 1;
		bIndex_ = 2;
		gbIndex_ = 3;
		break;
	case BayerFormat::Order::GBRG:
		gbIndex_ = 0;
		bIndex_ = 1;
		rIndex_ = 2;
		grIndex_ = 3;
		break;
	case BayerFormat::Order::BGGR:
		bIndex_ = 0;
		gbIndex_ = 1;
		grIndex_ = 2;
		rIndex_ = 3;
		break;
	default:
		LOG(MaliC55Agc, Error)
			<< "Invalid bayer format " << bayerOrder;
		return -EINVAL;
	}

	return 0;
}

void AgcStatistics::parseStatistics(const mali_c55_stats_buffer *stats)
{
	uint32_t r[256], g[256], b[256], y[256];

	/*
	 * We need to decode the bin values for each histogram from their 16-bit
	 * compressed values to a 32-bit value. We also take the average of the
	 * Gr/Gb values into a single green histogram.
	 */
	for (unsigned int i = 0; i < 256; i++) {
		r[i] = decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * rIndex_)]);
		g[i] = (decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * grIndex_)]) +
			decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * gbIndex_)])) / 2;
		b[i] = decodeBinValue(stats->ae_1024bin_hist.bins[i + (256 * bIndex_)]);

		y[i] = rec601LuminanceFromRGB({ { static_cast<double>(r[i]),
						  static_cast<double>(g[i]),
						  static_cast<double>(b[i]) } });
	}

	rHist = Histogram(Span<uint32_t>(r, kNumHistogramBins));
	gHist = Histogram(Span<uint32_t>(g, kNumHistogramBins));
	bHist = Histogram(Span<uint32_t>(b, kNumHistogramBins));
	yHist = Histogram(Span<uint32_t>(y, kNumHistogramBins));
}

Agc::Agc()
	: AgcMeanLuminance()
{
}

int Agc::init(IPAContext &context, const YamlObject &tuningData)
{
	int ret = parseTuningData(tuningData);
	if (ret)
		return ret;

	context.ctrlMap[&controls::AeEnable] = ControlInfo(false, true);
	context.ctrlMap[&controls::DigitalGain] = ControlInfo(
		static_cast<float>(kMinDigitalGain),
		static_cast<float>(kMaxDigitalGain),
		static_cast<float>(kMinDigitalGain)
	);
	context.ctrlMap.merge(controls());

	return 0;
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	int ret = statistics_.setBayerOrderIndices(context.configuration.sensor.bayerOrder);
	if (ret)
		return ret;

	/*
	 * Defaults; we use whatever the sensor's default exposure is and the
	 * minimum analogue gain. AEGC is _active_ by default.
	 */
	context.activeState.agc.autoEnabled = true;
	context.activeState.agc.automatic.sensorGain = context.configuration.agc.minAnalogueGain;
	context.activeState.agc.automatic.exposure = context.configuration.agc.defaultExposure;
	context.activeState.agc.automatic.ispGain = kMinDigitalGain;
	context.activeState.agc.manual.sensorGain = context.configuration.agc.minAnalogueGain;
	context.activeState.agc.manual.exposure = context.configuration.agc.defaultExposure;
	context.activeState.agc.manual.ispGain = kMinDigitalGain;
	context.activeState.agc.constraintMode = constraintModes().begin()->first;
	context.activeState.agc.exposureMode = exposureModeHelpers().begin()->first;

	/* \todo Run this again when FrameDurationLimits is passed in */
	setLimits(context.configuration.agc.minShutterSpeed,
		  context.configuration.agc.maxShutterSpeed,
		  context.configuration.agc.minAnalogueGain,
		  context.configuration.agc.maxAnalogueGain);

	resetFrameCount();

	return 0;
}

void Agc::queueRequest(IPAContext &context, const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &agc = context.activeState.agc;

	const auto &constraintMode = controls.get(controls::AeConstraintMode);
	agc.constraintMode = constraintMode.value_or(agc.constraintMode);

	const auto &exposureMode = controls.get(controls::AeExposureMode);
	agc.exposureMode = exposureMode.value_or(agc.exposureMode);

	const auto &agcEnable = controls.get(controls::AeEnable);
	if (agcEnable && *agcEnable != agc.autoEnabled) {
		agc.autoEnabled = *agcEnable;

		LOG(MaliC55Agc, Info)
			<< (agc.autoEnabled ? "Enabling" : "Disabling")
			<< " AGC";
	}

	/*
	 * If the automatic exposure and gain is enabled we have no further work
	 * to do here...
	 */
	if (agc.autoEnabled)
		return;

	/*
	 * ...otherwise we need to look for exposure and gain controls and use
	 * those to set the activeState.
	 */
	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure) {
		agc.manual.exposure = *exposure * 1.0us / context.configuration.sensor.lineDuration;

		LOG(MaliC55Agc, Debug)
			<< "Exposure set to " << agc.manual.exposure
			<< " on request sequence " << frame;
	}

	const auto &analogueGain = controls.get(controls::AnalogueGain);
	if (analogueGain) {
		agc.manual.sensorGain = *analogueGain;

		LOG(MaliC55Agc, Debug)
			<< "Analogue gain set to " << agc.manual.sensorGain
			<< " on request sequence " << frame;
	}

	const auto &digitalGain = controls.get(controls::DigitalGain);
	if (digitalGain) {
		agc.manual.ispGain = *digitalGain;

		LOG(MaliC55Agc, Debug)
			<< "Digital gain set to " << agc.manual.ispGain
			<< " on request sequence " << frame;
	}
}

size_t Agc::fillGainParamBlock(IPAContext &context, IPAFrameContext &frameContext,
			       mali_c55_params_block block)
{
	IPAActiveState &activeState = context.activeState;
	double gain;

	if (activeState.agc.autoEnabled)
		gain = activeState.agc.automatic.ispGain;
	else
		gain = activeState.agc.manual.ispGain;

	block.header->type = MALI_C55_PARAM_BLOCK_DIGITAL_GAIN;
	block.header->flags = MALI_C55_PARAM_BLOCK_FL_NONE;
	block.header->size = sizeof(struct mali_c55_params_digital_gain);

	block.digital_gain->gain = floatingToFixedPoint<5, 8, uint16_t, double>(gain);
	frameContext.agc.ispGain = gain;

	return block.header->size;
}

size_t Agc::fillParamsBuffer(mali_c55_params_block block,
			     enum mali_c55_param_block_type type)
{
	block.header->type = type;
	block.header->flags = MALI_C55_PARAM_BLOCK_FL_NONE;
	block.header->size = sizeof(struct mali_c55_params_aexp_hist);

	/* Collect every 3rd pixel horizontally */
	block.aexp_hist->skip_x = 1;
	/* Start from first column */
	block.aexp_hist->offset_x = 0;
	/* Collect every pixel vertically */
	block.aexp_hist->skip_y = 0;
	/* Start from the first row */
	block.aexp_hist->offset_y = 0;
	/* 1x scaling (i.e. none) */
	block.aexp_hist->scale_bottom = 0;
	block.aexp_hist->scale_top = 0;
	/* Collect all Bayer planes into 4 separate histograms */
	block.aexp_hist->plane_mode = 1;
	/* Tap the data immediately after the digital gain block */
	block.aexp_hist->tap_point = MALI_C55_AEXP_HIST_TAP_FS;

	return block.header->size;
}

size_t Agc::fillWeightsArrayBuffer(mali_c55_params_block block,
				   enum mali_c55_param_block_type type)
{
	block.header->type = type;
	block.header->flags = MALI_C55_PARAM_BLOCK_FL_NONE;
	block.header->size = sizeof(struct mali_c55_params_aexp_weights);

	/* We use every zone - a 15x15 grid */
	block.aexp_weights->nodes_used_horiz = 15;
	block.aexp_weights->nodes_used_vert = 15;

	/*
	 * We uniformly weight the zones to 1 - this results in the collected
	 * histograms containing a true pixel count, which we can then use to
	 * approximate colour channel averages for the image.
	 */
	Span<uint8_t> weights{
		block.aexp_weights->zone_weights,
		MALI_C55_MAX_ZONES
	};
	std::fill(weights.begin(), weights.end(), 1);

	return block.header->size;
}

void Agc::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, mali_c55_params_buffer *params)
{
	mali_c55_params_block block;

	block.data = &params->data[params->total_size];
	params->total_size += fillGainParamBlock(context, frameContext, block);

	if (frame > 0)
		return;

	block.data = &params->data[params->total_size];
	params->total_size += fillParamsBuffer(block,
					       MALI_C55_PARAM_BLOCK_AEXP_HIST);

	block.data = &params->data[params->total_size];
	params->total_size += fillWeightsArrayBuffer(block,
						     MALI_C55_PARAM_BLOCK_AEXP_HIST_WEIGHTS);

	block.data = &params->data[params->total_size];
	params->total_size += fillParamsBuffer(block,
					       MALI_C55_PARAM_BLOCK_AEXP_IHIST);

	block.data = &params->data[params->total_size];
	params->total_size += fillWeightsArrayBuffer(block,
						     MALI_C55_PARAM_BLOCK_AEXP_IHIST_WEIGHTS);
}

double Agc::estimateLuminance(const double gain) const
{
	double rAvg = statistics_.rHist.interQuantileMean(0, 1) * gain;
	double gAvg = statistics_.gHist.interQuantileMean(0, 1) * gain;
	double bAvg = statistics_.bHist.interQuantileMean(0, 1) * gain;
	double yAvg = rec601LuminanceFromRGB({ { rAvg, gAvg, bAvg } });

	return yAvg / kNumHistogramBins;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const mali_c55_stats_buffer *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	IPASessionConfiguration &configuration = context.configuration;
	IPAActiveState &activeState = context.activeState;

	if (!stats) {
		LOG(MaliC55Agc, Error) << "No statistics buffer passed to Agc";
		return;
	}

	statistics_.parseStatistics(stats);
	context.activeState.agc.temperatureK = estimateCCT({ { statistics_.rHist.interQuantileMean(0, 1),
							       statistics_.gHist.interQuantileMean(0, 1),
							       statistics_.bHist.interQuantileMean(0, 1) } });

	/*
	 * The Agc algorithm needs to know the effective exposure value that was
	 * applied to the sensor when the statistics were collected.
	 */
	uint32_t exposure = frameContext.agc.exposure;
	double analogueGain = frameContext.agc.sensorGain;
	double digitalGain = frameContext.agc.ispGain;
	double totalGain = analogueGain * digitalGain;
	utils::Duration currentShutter = exposure * configuration.sensor.lineDuration;
	utils::Duration effectiveExposureValue = currentShutter * totalGain;

	utils::Duration shutterTime;
	double aGain, dGain;
	std::tie(shutterTime, aGain, dGain) =
		calculateNewEv(activeState.agc.constraintMode,
			       activeState.agc.exposureMode, statistics_.yHist,
			       effectiveExposureValue);

	dGain = std::clamp(dGain, kMinDigitalGain, kMaxDigitalGain);

	LOG(MaliC55Agc, Debug)
		<< "Divided up shutter, analogue gain and digital gain are "
		<< shutterTime << ", " << aGain << " and " << dGain;

	activeState.agc.automatic.exposure = shutterTime / configuration.sensor.lineDuration;
	activeState.agc.automatic.sensorGain = aGain;
	activeState.agc.automatic.ispGain = dGain;

	metadata.set(controls::ExposureTime, currentShutter.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.agc.sensorGain);
	metadata.set(controls::DigitalGain, frameContext.agc.ispGain);
	metadata.set(controls::ColourTemperature, context.activeState.agc.temperatureK);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
