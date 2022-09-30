/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * dpcc.cpp - RkISP1 Defect Pixel Cluster Correction control
 */

#include "dpcc.h"

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "linux/rkisp1-config.h"

/**
 * \file dpcc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class DefectPixelClusterCorrection
 * \brief RkISP1 Defect Pixel Cluster Correction control
 *
 * Depending of the sensor quality, some pixels can be defective and then
 * appear significantly brighter or darker than the other pixels.
 *
 * The Defect Pixel Cluster Correction algorithms is responsible to minimize
 * the impact of the pixels. This can be done with algorithms applied at run
 * time (on-the-fly method) or with a table of defective pixels. Only the first
 * method is supported for the moment.
 */

LOG_DEFINE_CATEGORY(RkISP1Dpcc)

DefectPixelClusterCorrection::DefectPixelClusterCorrection()
	: config_({})
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int DefectPixelClusterCorrection::init([[maybe_unused]] IPAContext &context,
				       const YamlObject &tuningData)
{
	config_.mode = RKISP1_CIF_ISP_DPCC_MODE_STAGE1_ENABLE;
	config_.output_mode = RKISP1_CIF_ISP_DPCC_OUTPUT_MODE_STAGE1_INCL_G_CENTER
			    | RKISP1_CIF_ISP_DPCC_OUTPUT_MODE_STAGE1_INCL_RB_CENTER;

	config_.set_use = tuningData["fixed-set"].get<bool>(false)
			? RKISP1_CIF_ISP_DPCC_SET_USE_STAGE1_USE_FIX_SET : 0;

	/* Get all defined sets to apply (up to 3). */
	const YamlObject &setsObject = tuningData["sets"];
	if (!setsObject.isList()) {
		LOG(RkISP1Dpcc, Error)
			<< "'sets' parameter not found in tuning file";
		return -EINVAL;
	}

	if (setsObject.size() > RKISP1_CIF_ISP_DPCC_METHODS_MAX) {
		LOG(RkISP1Dpcc, Error)
			<< "'sets' size in tuning file (" << setsObject.size()
			<< ") exceeds the maximum hardware capacity (3)";
		return -EINVAL;
	}

	for (std::size_t i = 0; i < setsObject.size(); ++i) {
		struct rkisp1_cif_isp_dpcc_methods_config &method = config_.methods[i];
		const YamlObject &set = setsObject[i];
		uint16_t value;

		/* Enable set if described in YAML tuning file. */
		config_.set_use |= 1 << i;

		/* PG Method */
		const YamlObject &pgObject = set["pg-factor"];

		if (pgObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_PG_GREEN_ENABLE;

			value = pgObject["green"].get<uint16_t>(0);
			method.pg_fac |= RKISP1_CIF_ISP_DPCC_PG_FAC_G(value);
		}

		if (pgObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_PG_RED_BLUE_ENABLE;

			value = pgObject["red-blue"].get<uint16_t>(0);
			method.pg_fac |= RKISP1_CIF_ISP_DPCC_PG_FAC_RB(value);
		}

		/* RO Method */
		const YamlObject &roObject = set["ro-limits"];

		if (roObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RO_GREEN_ENABLE;

			value = roObject["green"].get<uint16_t>(0);
			config_.ro_limits |=
				RKISP1_CIF_ISP_DPCC_RO_LIMITS_n_G(i, value);
		}

		if (roObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RO_RED_BLUE_ENABLE;

			value = roObject["red-blue"].get<uint16_t>(0);
			config_.ro_limits |=
				RKISP1_CIF_ISP_DPCC_RO_LIMITS_n_RB(i, value);
		}

		/* RG Method */
		const YamlObject &rgObject = set["rg-factor"];
		method.rg_fac = 0;

		if (rgObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RG_GREEN_ENABLE;

			value = rgObject["green"].get<uint16_t>(0);
			method.rg_fac |= RKISP1_CIF_ISP_DPCC_RG_FAC_G(value);
		}

		if (rgObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RG_RED_BLUE_ENABLE;

			value = rgObject["red-blue"].get<uint16_t>(0);
			method.rg_fac |= RKISP1_CIF_ISP_DPCC_RG_FAC_RB(value);
		}

		/* RND Method */
		const YamlObject &rndOffsetsObject = set["rnd-offsets"];

		if (rndOffsetsObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RND_GREEN_ENABLE;

			value = rndOffsetsObject["green"].get<uint16_t>(0);
			config_.rnd_offs |=
				RKISP1_CIF_ISP_DPCC_RND_OFFS_n_G(i, value);
		}

		if (rndOffsetsObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RND_RED_BLUE_ENABLE;

			value = rndOffsetsObject["red-blue"].get<uint16_t>(0);
			config_.rnd_offs |=
				RKISP1_CIF_ISP_DPCC_RND_OFFS_n_RB(i, value);
		}

		const YamlObject &rndThresholdObject = set["rnd-threshold"];
		method.rnd_thresh = 0;

		if (rndThresholdObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RND_GREEN_ENABLE;

			value = rndThresholdObject["green"].get<uint16_t>(0);
			method.rnd_thresh |=
				RKISP1_CIF_ISP_DPCC_RND_THRESH_G(value);
		}

		if (rndThresholdObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_RND_RED_BLUE_ENABLE;

			value = rndThresholdObject["red-blue"].get<uint16_t>(0);
			method.rnd_thresh |=
				RKISP1_CIF_ISP_DPCC_RND_THRESH_RB(value);
		}

		/* LC Method */
		const YamlObject &lcThresholdObject = set["line-threshold"];
		method.line_thresh = 0;

		if (lcThresholdObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_LC_GREEN_ENABLE;

			value = lcThresholdObject["green"].get<uint16_t>(0);
			method.line_thresh |=
				RKISP1_CIF_ISP_DPCC_LINE_THRESH_G(value);
		}

		if (lcThresholdObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_LC_RED_BLUE_ENABLE;

			value = lcThresholdObject["red-blue"].get<uint16_t>(0);
			method.line_thresh |=
				RKISP1_CIF_ISP_DPCC_LINE_THRESH_RB(value);
		}

		const YamlObject &lcTMadFactorObject = set["line-mad-factor"];
		method.line_mad_fac = 0;

		if (lcTMadFactorObject.contains("green")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_LC_GREEN_ENABLE;

			value = lcTMadFactorObject["green"].get<uint16_t>(0);
			method.line_mad_fac |=
				RKISP1_CIF_ISP_DPCC_LINE_MAD_FAC_G(value);
		}

		if (lcTMadFactorObject.contains("red-blue")) {
			method.method |=
				RKISP1_CIF_ISP_DPCC_METHODS_SET_LC_RED_BLUE_ENABLE;

			value = lcTMadFactorObject["red-blue"].get<uint16_t>(0);
			method.line_mad_fac |=
				RKISP1_CIF_ISP_DPCC_LINE_MAD_FAC_RB(value);
		}
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void DefectPixelClusterCorrection::prepare([[maybe_unused]] IPAContext &context,
					   const uint32_t frame,
					   [[maybe_unused]] IPAFrameContext &frameContext,
					   rkisp1_params_cfg *params)
{
	if (frame > 0)
		return;

	params->others.dpcc_config = config_;

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_DPCC;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_DPCC;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_DPCC;
}

REGISTER_IPA_ALGORITHM(DefectPixelClusterCorrection, "DefectPixelClusterCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
