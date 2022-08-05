/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * lsc.cpp - RkISP1 Lens Shading Correction control
 */

#include "lsc.h"

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
		tuningData[prop].getList<double>().value_or(utils::defopt);
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
	float sum = std::accumulate(sizes.begin(), sizes.end(), 0.0f);
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
		tuningData[prop].getList<uint16_t>().value_or(utils::defopt);
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
	: initialized_(false)
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

	rData_ = parseTable(tuningData, "r");
	grData_ = parseTable(tuningData, "gr");
	gbData_ = parseTable(tuningData, "gb");
	bData_ = parseTable(tuningData, "b");

	if (rData_.empty() || grData_.empty() ||
	    gbData_.empty() || bData_.empty())
		return -EINVAL;

	initialized_ = true;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int LensShadingCorrection::configure(IPAContext &context,
				     [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	context.configuration.lsc.enabled = initialized_;
	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare(IPAContext &context,
				    rkisp1_params_cfg *params)
{
	if (context.frameContext.frameCount > 0)
		return;

	if (!initialized_)
		return;

	struct rkisp1_cif_isp_lsc_config &config = params->others.lsc_config;
	const Size &size = context.configuration.sensor.size;
	Size totalSize{};

	for (unsigned int i = 0; i < RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE; ++i) {
		config.x_size_tbl[i] = xSize_[i] * size.width;
		config.y_size_tbl[i] = ySize_[i] * size.height;

		/*
		 * To prevent unexpected behavior of the ISP, the sum of x_size_tbl and
		 * y_size_tbl items shall be equal to respectively size.width/2 and
		 * size.height/2. Enforce it by computing the last tables value to avoid
		 * rounding-induced errors.
		 */
		if (i == RKISP1_CIF_ISP_LSC_SECTORS_TBL_SIZE - 1) {
			config.x_size_tbl[i] = size.width / 2 - totalSize.width;
			config.y_size_tbl[i] = size.height / 2 - totalSize.height;
		}

		totalSize.width += config.x_size_tbl[i];
		totalSize.height += config.y_size_tbl[i];

		config.x_grad_tbl[i] = std::round(32768 / config.x_size_tbl[i]);
		config.y_grad_tbl[i] = std::round(32768 / config.y_size_tbl[i]);
	}

	std::copy(rData_.begin(), rData_.end(),
		  &params->others.lsc_config.r_data_tbl[0][0]);
	std::copy(grData_.begin(), grData_.end(),
		  &params->others.lsc_config.gr_data_tbl[0][0]);
	std::copy(gbData_.begin(), gbData_.end(),
		  &params->others.lsc_config.gb_data_tbl[0][0]);
	std::copy(bData_.begin(), bData_.end(),
		  &params->others.lsc_config.b_data_tbl[0][0]);

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_LSC;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_LSC;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_LSC;
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
