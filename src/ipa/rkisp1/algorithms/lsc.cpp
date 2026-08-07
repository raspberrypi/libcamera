/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Lens Shading Correction control
 */

#include "lsc.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

/**
 * \file lsc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

LOG_DEFINE_CATEGORY(RkISP1Lsc)

namespace {

constexpr int kColourTemperatureQuantization = 10;

std::vector<double> parseSizes(const ValueNode &tuningData,
			       const char *prop)
{
	std::vector<double> sizes =
		tuningData[prop].get<std::vector<double>>().value_or(utils::defopt);
	if (sizes.size() != RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE) {
		LOG(RkISP1Lsc, Error)
			<< "Invalid '" << prop << "' values: expected "
			<< RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE
			<< " elements, got " << sizes.size();
		return {};
	}

	/*
	 * The sum of all elements must be 0.5 to satisfy hardware constraints.
	 * Validate it here, allowing a 1% tolerance as rounding errors may
	 * prevent an exact match (further adjustments will be performed in
	 * LensShadingCorrection::prepare()).
	 */
	double sum = std::accumulate(sizes.begin(), sizes.end(), 0.0);
	if (sum < 0.495 || sum > 0.505) {
		LOG(RkISP1Lsc, Error)
			<< "Invalid '" << prop << "' values: sum of the elements"
			<< " should be 0.5, got " << sum;
		return {};
	}

	return sizes;
}

/*
 * The rkisp1 LSC grid spacing is defined by the cell sizes on the top-left
 * quadrant of the grid. This is then mirrored in hardware to the other
 * quadrants. See parseSizes() for further details. For easier handling, this
 * function converts the cell sizes of half the grid to a list of position of
 * the whole grid (on one axis). Example:
 *
 * input:   | 0.2 | 0.3 |
 * output: 0.0   0.2   0.5   0.8   1.0
 */
std::vector<double> sizesListToPositions(Span<const double> sizes)
{
	const int half = sizes.size();
	std::vector<double> positions(half * 2 + 1);
	double x = 0.0;

	positions[half] = 0.5;
	for (int i = 1; i <= half; i++) {
		x += sizes[half - i];
		positions[half - i] = 0.5 - x;
		positions[half + i] = 0.5 + x;
	}

	return positions;
}

unsigned int quantize(unsigned int value, unsigned int step)
{
	return std::lround(value / static_cast<double>(step)) * step;
}

} /* namespace */

/**
 * \class LensShadingCorrection
 * \brief RkISP1 Lens Shading Correction control
 *
 * Due to the optical characteristics of the lens, the light intensity received
 * by the sensor is not uniform.
 *
 * The Lens Shading Correction algorithm applies multipliers to all pixels to
 * compensate for the lens shading effect. The coefficients are specified in a
 * downscaled table in the tuning data.
 */

LensShadingCorrection::LensShadingCorrection()
	: lastAppliedCt_(0), lastAppliedQuantizedCt_(0)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int LensShadingCorrection::init([[maybe_unused]] IPAContext &context,
				const ValueNode &tuningData)
{
	xSize_ = parseSizes(tuningData, "x-size");
	ySize_ = parseSizes(tuningData, "y-size");

	if (xSize_.empty() || ySize_.empty())
		return -EINVAL;

	xPos_ = sizesListToPositions(xSize_);
	yPos_ = sizesListToPositions(ySize_);

	return lscAlgo_.init(tuningData,  context.ctrlMap, {
				.keys = { "r", "gr", "gb", "b" },
				.numHSamples = RKISP1_CIF_ISP_LSC_SAMPLES_MAX,
				.numVSamples = RKISP1_CIF_ISP_LSC_SAMPLES_MAX,
				.sensorSize = context.sensorInfo.activeAreaSize
			     });
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int LensShadingCorrection::configure(IPAContext &context,
				     [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	const Size &size = context.configuration.sensor.size;
	Size totalSize{};

	for (unsigned int i = 0; i < RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE; ++i) {
		xSizes_[i] = xSize_[i] * size.width;
		ySizes_[i] = ySize_[i] * size.height;

		/*
		 * To prevent unexpected behavior of the ISP, the sum of x_size_tbl and
		 * y_size_tbl items shall be equal to respectively size.width/2 and
		 * size.height/2. Enforce it by computing the last tables value to avoid
		 * rounding-induced errors.
		 */
		if (i == RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE - 1) {
			xSizes_[i] = size.width / 2 - totalSize.width;
			ySizes_[i] = size.height / 2 - totalSize.height;
		}

		totalSize.width += xSizes_[i];
		totalSize.height += ySizes_[i];

		xGrad_[i] = std::round(32768 / xSizes_[i]);
		yGrad_[i] = std::round(32768 / ySizes_[i]);
	}

	return lscAlgo_.configure(context.activeState.lsc, configInfo.analogCrop,
				  xPos_, yPos_);
}

void LensShadingCorrection::setParameters(rkisp1_cif_isp_lsc_config &config)
{
	memcpy(config.x_grad_tbl, xGrad_, sizeof(config.x_grad_tbl));
	memcpy(config.y_grad_tbl, yGrad_, sizeof(config.y_grad_tbl));
	memcpy(config.x_size_tbl, xSizes_, sizeof(config.x_size_tbl));
	memcpy(config.y_size_tbl, ySizes_, sizeof(config.y_size_tbl));
}

void LensShadingCorrection::copyTable(rkisp1_cif_isp_lsc_config &config,
				      const lsc::Components<uint16_t> &set)
{
	const auto &r = set.at("r");
	std::copy(r.begin(), r.end(), &config.r_data_tbl[0][0]);
	const auto &gr = set.at("gr");
	std::copy(gr.begin(), gr.end(), &config.gr_data_tbl[0][0]);
	const auto &gb = set.at("gb");
	std::copy(gb.begin(), gb.end(), &config.gb_data_tbl[0][0]);
	const auto &b = set.at("b");
	std::copy(b.begin(), b.end(), &config.b_data_tbl[0][0]);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void LensShadingCorrection::queueRequest(IPAContext &context,
					 [[maybe_unused]] const uint32_t frame,
					 IPAFrameContext &frameContext,
					 const ControlList &controls)
{
	lscAlgo_.queueRequest(context.activeState.lsc, frameContext.lsc,
			      controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare([[maybe_unused]] IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    IPAFrameContext &frameContext,
				    RkISP1Params *params)
{
	uint32_t ct = frameContext.awb.colourTemperature;
	unsigned int quantizedCt = quantize(ct, kColourTemperatureQuantization);

	/* Check if we can skip the update. */
	if (!frameContext.lsc.update) {
		if (!frameContext.lsc.enabled)
			return;

		/*
		 * Add a threshold so that oscillations around a quantization
		 * step don't lead to constant changes.
		 */
		if (utils::abs_diff(ct, lastAppliedCt_) < kColourTemperatureQuantization / 2)
			return;

		if (quantizedCt == lastAppliedQuantizedCt_)
			return;
	}

	auto config = params->block<BlockType::Lsc>();
	config.setEnabled(frameContext.lsc.enabled);

	if (!frameContext.lsc.enabled)
		return;

	setParameters(*config);

	copyTable(*config, lscAlgo_.interpolateComponents(quantizedCt));

	lastAppliedCt_ = ct;
	lastAppliedQuantizedCt_ = quantizedCt;

	LOG(RkISP1Lsc, Debug)
		<< "ct is " << ct << ", quantized to "
		<< quantizedCt;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void LensShadingCorrection::process([[maybe_unused]] IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    IPAFrameContext &frameContext,
				    [[maybe_unused]] const rkisp1_stat_buffer *stats,
				    ControlList &metadata)
{
	lscAlgo_.process(frameContext.lsc, metadata);
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
