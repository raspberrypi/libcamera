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

#include "libcamera/internal/yaml_parser.h"

#include "linux/rkisp1-config.h"

/**
 * \file lsc.h
 */

namespace libcamera {

namespace ipa {

constexpr int kColourTemperatureChangeThreshhold = 10;

template<typename T>
void interpolateVector(const std::vector<T> &a, const std::vector<T> &b,
		       std::vector<T> &dest, double lambda)
{
	assert(a.size() == b.size());
	dest.resize(a.size());
	for (size_t i = 0; i < a.size(); i++) {
		dest[i] = a[i] * (1.0 - lambda) + b[i] * lambda;
	}
}

template<>
void Interpolator<rkisp1::algorithms::LensShadingCorrection::Components>::
	interpolate(const rkisp1::algorithms::LensShadingCorrection::Components &a,
		    const rkisp1::algorithms::LensShadingCorrection::Components &b,
		    rkisp1::algorithms::LensShadingCorrection::Components &dest,
		    double lambda)
{
	interpolateVector(a.r, b.r, dest.r, lambda);
	interpolateVector(a.gr, b.gr, dest.gr, lambda);
	interpolateVector(a.gb, b.gb, dest.gb, lambda);
	interpolateVector(a.b, b.b, dest.b, lambda);
}

} /* namespace ipa */

namespace ipa::rkisp1::algorithms {

/**
 * \class LensShadingCorrection
 * \brief RkISP1 Lens Shading Correction control
 *
 * Due to the optical characteristics of the lens, the light intensity received
 * by the sensor is not uniform.
 *
 * The Lens Shading Correction algorithm applies multipliers to all pixels
 * to compensate for the lens shading effect. The coefficients are
 * specified in a downscaled table in the YAML tuning file.
 */

LOG_DEFINE_CATEGORY(RkISP1Lsc)

static std::vector<double> parseSizes(const YamlObject &tuningData,
				      const char *prop)
{
	std::vector<double> sizes =
		tuningData[prop].getList<double>().value_or(std::vector<double>{});
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

static std::vector<uint16_t> parseTable(const YamlObject &tuningData,
					const char *prop)
{
	static constexpr unsigned int kLscNumSamples =
		RKISP1_CIF_ISP_LSC_SAMPLES_MAX * RKISP1_CIF_ISP_LSC_SAMPLES_MAX;

	std::vector<uint16_t> table =
		tuningData[prop].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (table.size() != kLscNumSamples) {
		LOG(RkISP1Lsc, Error)
			<< "Invalid '" << prop << "' values: expected "
			<< kLscNumSamples
			<< " elements, got " << table.size();
		return {};
	}

	return table;
}

LensShadingCorrection::LensShadingCorrection()
	: lastAppliedCt_(0), lastAppliedQuantizedCt_(0)
{
	sets_.setQuantization(kColourTemperatureChangeThreshhold);
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int LensShadingCorrection::init([[maybe_unused]] IPAContext &context,
				const YamlObject &tuningData)
{
	xSize_ = parseSizes(tuningData, "x-size");
	ySize_ = parseSizes(tuningData, "y-size");

	if (xSize_.empty() || ySize_.empty())
		return -EINVAL;

	/* Get all defined sets to apply. */
	const YamlObject &yamlSets = tuningData["sets"];
	if (!yamlSets.isList()) {
		LOG(RkISP1Lsc, Error)
			<< "'sets' parameter not found in tuning file";
		return -EINVAL;
	}

	const auto &sets = yamlSets.asList();
	std::map<unsigned int, Components> lscData;
	for (const auto &yamlSet : sets) {
		uint32_t ct = yamlSet["ct"].get<uint32_t>(0);

		if (lscData.count(ct)) {
			LOG(RkISP1Lsc, Error)
				<< "Multiple sets found for color temperature "
				<< ct;
			return -EINVAL;
		}

		Components &set = lscData[ct];

		set.ct = ct;
		set.r = parseTable(yamlSet, "r");
		set.gr = parseTable(yamlSet, "gr");
		set.gb = parseTable(yamlSet, "gb");
		set.b = parseTable(yamlSet, "b");

		if (set.r.empty() || set.gr.empty() ||
		    set.gb.empty() || set.b.empty()) {
			LOG(RkISP1Lsc, Error)
				<< "Set for color temperature " << ct
				<< " is missing tables";
			return -EINVAL;
		}
	}

	if (lscData.empty()) {
		LOG(RkISP1Lsc, Error) << "Failed to load any sets";
		return -EINVAL;
	}

	sets_.setData(std::move(lscData));

	return 0;
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

	context.configuration.lsc.enabled = true;
	return 0;
}

void LensShadingCorrection::setParameters(rkisp1_cif_isp_lsc_config &config)
{
	memcpy(config.x_grad_tbl, xGrad_, sizeof(config.x_grad_tbl));
	memcpy(config.y_grad_tbl, yGrad_, sizeof(config.y_grad_tbl));
	memcpy(config.x_size_tbl, xSizes_, sizeof(config.x_size_tbl));
	memcpy(config.y_size_tbl, ySizes_, sizeof(config.y_size_tbl));
}

void LensShadingCorrection::copyTable(rkisp1_cif_isp_lsc_config &config,
				      const Components &set)
{
	std::copy(set.r.begin(), set.r.end(), &config.r_data_tbl[0][0]);
	std::copy(set.gr.begin(), set.gr.end(), &config.gr_data_tbl[0][0]);
	std::copy(set.gb.begin(), set.gb.end(), &config.gb_data_tbl[0][0]);
	std::copy(set.b.begin(), set.b.end(), &config.b_data_tbl[0][0]);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare(IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    [[maybe_unused]] IPAFrameContext &frameContext,
				    RkISP1Params *params)
{
	uint32_t ct = context.activeState.awb.temperatureK;
	if (std::abs(static_cast<int>(ct) - static_cast<int>(lastAppliedCt_)) <
	    kColourTemperatureChangeThreshhold)
		return;
	unsigned int quantizedCt;
	const Components &set = sets_.getInterpolated(ct, &quantizedCt);
	if (lastAppliedQuantizedCt_ == quantizedCt)
		return;

	auto config = params->block<BlockType::Lsc>();
	config.setEnabled(true);
	setParameters(*config);
	copyTable(*config, set);

	lastAppliedCt_ = ct;
	lastAppliedQuantizedCt_ = quantizedCt;

	LOG(RkISP1Lsc, Debug)
		<< "ct is " << ct << ", quantized to "
		<< quantizedCt;
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
