/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * lsc.cpp - RkISP1 Lens Shading Correction control
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
	: lastCt_({ 0, 0 })
{
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
	for (const auto &yamlSet : sets) {
		uint32_t ct = yamlSet["ct"].get<uint32_t>(0);

		if (sets_.count(ct)) {
			LOG(RkISP1Lsc, Error)
				<< "Multiple sets found for color temperature "
				<< ct;
			return -EINVAL;
		}

		Components &set = sets_[ct];

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

	if (sets_.empty()) {
		LOG(RkISP1Lsc, Error) << "Failed to load any sets";
		return -EINVAL;
	}

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

void LensShadingCorrection::setParameters(rkisp1_params_cfg *params)
{
	struct rkisp1_cif_isp_lsc_config &config = params->others.lsc_config;

	memcpy(config.x_grad_tbl, xGrad_, sizeof(config.x_grad_tbl));
	memcpy(config.y_grad_tbl, yGrad_, sizeof(config.y_grad_tbl));
	memcpy(config.x_size_tbl, xSizes_, sizeof(config.x_size_tbl));
	memcpy(config.y_size_tbl, ySizes_, sizeof(config.y_size_tbl));

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_LSC;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_LSC;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_LSC;
}

void LensShadingCorrection::copyTable(rkisp1_cif_isp_lsc_config &config,
				      const Components &set)
{
	std::copy(set.r.begin(), set.r.end(), &config.r_data_tbl[0][0]);
	std::copy(set.gr.begin(), set.gr.end(), &config.gr_data_tbl[0][0]);
	std::copy(set.gb.begin(), set.gb.end(), &config.gb_data_tbl[0][0]);
	std::copy(set.b.begin(), set.b.end(), &config.b_data_tbl[0][0]);
}

/*
 * Interpolate LSC parameters based on color temperature value.
 */
void LensShadingCorrection::interpolateTable(rkisp1_cif_isp_lsc_config &config,
					     const Components &set0,
					     const Components &set1,
					     const uint32_t ct)
{
	double coeff0 = (set1.ct - ct) / (set1.ct - set0.ct);
	double coeff1 = (ct - set0.ct) / (set1.ct - set0.ct);

	for (unsigned int i = 0; i < RKISP1_CIF_ISP_LSC_SAMPLES_MAX; ++i) {
		for (unsigned int j = 0; j < RKISP1_CIF_ISP_LSC_SAMPLES_MAX; ++j) {
			unsigned int sample = i * RKISP1_CIF_ISP_LSC_SAMPLES_MAX + j;

			config.r_data_tbl[i][j] =
				set0.r[sample] * coeff0 +
				set1.r[sample] * coeff1;

			config.gr_data_tbl[i][j] =
				set0.gr[sample] * coeff0 +
				set1.gr[sample] * coeff1;

			config.gb_data_tbl[i][j] =
				set0.gb[sample] * coeff0 +
				set1.gb[sample] * coeff1;

			config.b_data_tbl[i][j] =
				set0.b[sample] * coeff0 +
				set1.b[sample] * coeff1;
		}
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare(IPAContext &context,
				    const uint32_t frame,
				    [[maybe_unused]] IPAFrameContext &frameContext,
				    rkisp1_params_cfg *params)
{
	struct rkisp1_cif_isp_lsc_config &config = params->others.lsc_config;

	/*
	 * If there is only one set, the configuration has already been done
	 * for first frame.
	 */
	if (sets_.size() == 1 && frame > 0)
		return;

	/*
	 * If there is only one set, pick it. We can ignore lastCt_, as it will
	 * never be relevant.
	 */
	if (sets_.size() == 1) {
		setParameters(params);
		copyTable(config, sets_.cbegin()->second);
		return;
	}

	uint32_t ct = context.activeState.awb.temperatureK;
	ct = std::clamp(ct, sets_.cbegin()->first, sets_.crbegin()->first);

	/*
	 * If the original is the same, then it means the same adjustment would
	 * be made. If the adjusted is the same, then it means that it's the
	 * same as what was actually applied. Thus in these cases we can skip
	 * reprogramming the LSC.
	 *
	 * original == adjusted can only happen if an interpolation
	 * happened, or if original has an exact entry in sets_. This means
	 * that if original != adjusted, then original was adjusted to
	 * the nearest available entry in sets_, resulting in adjusted.
	 * Clearly, any ct value that is in between original and adjusted
	 * will be adjusted to the same adjusted value, so we can skip
	 * reprogramming the LSC table.
	 *
	 * We also skip updating the original value, as the last one had a
	 * larger bound and thus a larger range of ct values that will be
	 * adjusted to the same adjusted.
	 */
	if ((lastCt_.original <= ct && ct <= lastCt_.adjusted) ||
	    (lastCt_.adjusted <= ct && ct <= lastCt_.original))
		return;

	setParameters(params);

	/*
	 * The color temperature matches exactly one of the available LSC tables.
	 */
	if (sets_.count(ct)) {
		copyTable(config, sets_[ct]);
		lastCt_ = { ct, ct };
		return;
	}

	/* No shortcuts left; we need to round or interpolate */
	auto iter = sets_.upper_bound(ct);
	const Components &set1 = iter->second;
	const Components &set0 = (--iter)->second;
	uint32_t ct0 = set0.ct;
	uint32_t ct1 = set1.ct;
	uint32_t diff0 = ct - ct0;
	uint32_t diff1 = ct1 - ct;
	static constexpr double kThreshold = 0.1;
	float threshold = kThreshold * (ct1 - ct0);

	if (diff0 < threshold || diff1 < threshold) {
		const Components &set = diff0 < diff1 ? set0 : set1;
		LOG(RkISP1Lsc, Debug) << "using LSC table for " << set.ct;
		copyTable(config, set);
		lastCt_ = { ct, set.ct };
		return;
	}

	/*
	 * ct is not within 10% of the difference between the neighbouring
	 * color temperatures, so we need to interpolate.
	 */
	LOG(RkISP1Lsc, Debug)
		<< "ct is " << ct << ", interpolating between "
		<< ct0 << " and " << ct1;
	interpolateTable(config, set0, set1, ct);
	lastCt_ = { ct, ct };
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
