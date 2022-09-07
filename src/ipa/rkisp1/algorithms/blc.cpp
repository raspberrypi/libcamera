/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * blc.cpp - RkISP1 Black Level Correction control
 */

#include "blc.h"

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

/**
 * \file blc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class BlackLevelCorrection
 * \brief RkISP1 Black Level Correction control
 *
 * The pixels output by the camera normally include a black level, because
 * sensors do not always report a signal level of '0' for black. Pixels at or
 * below this level should be considered black. To achieve that, the RkISP BLC
 * algorithm subtracts a configurable offset from all pixels.
 *
 * The black level can be measured at runtime from an optical dark region of the
 * camera sensor, or measured during the camera tuning process. The first option
 * isn't currently supported.
 */

LOG_DEFINE_CATEGORY(RkISP1Blc)

BlackLevelCorrection::BlackLevelCorrection()
	: tuningParameters_(false)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int BlackLevelCorrection::init([[maybe_unused]] IPAContext &context,
			       const YamlObject &tuningData)
{
	blackLevelRed_ = tuningData["R"].get<int16_t>(256);
	blackLevelGreenR_ = tuningData["Gr"].get<int16_t>(256);
	blackLevelGreenB_ = tuningData["Gb"].get<int16_t>(256);
	blackLevelBlue_ = tuningData["B"].get<int16_t>(256);

	tuningParameters_ = true;

	LOG(RkISP1Blc, Debug)
		<< "Black levels: red " << blackLevelRed_
		<< ", green (red) " << blackLevelGreenR_
		<< ", green (blue) " << blackLevelGreenB_
		<< ", blue " << blackLevelBlue_;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare([[maybe_unused]] IPAContext &context,
				   const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   rkisp1_params_cfg *params)
{
	if (frame > 0)
		return;

	if (!tuningParameters_)
		return;

	params->others.bls_config.enable_auto = 0;
	params->others.bls_config.fixed_val.r = blackLevelRed_;
	params->others.bls_config.fixed_val.gr = blackLevelGreenR_;
	params->others.bls_config.fixed_val.gb = blackLevelGreenB_;
	params->others.bls_config.fixed_val.b = blackLevelBlue_;

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_BLS;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_BLS;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_BLS;
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
