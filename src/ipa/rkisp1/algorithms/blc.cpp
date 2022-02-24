/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * blc.cpp - RkISP1 Black Level Correction control
 */

#include "blc.h"

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

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare(IPAContext &context,
				   rkisp1_params_cfg *params)
{
	if (context.frameContext.frameCount > 0)
		return;
	/*
	 * Substract fixed values taken from imx219 tuning file.
	 * \todo Use a configuration file for it ?
	 */
	params->others.bls_config.enable_auto = 0;
	params->others.bls_config.fixed_val.r = 256;
	params->others.bls_config.fixed_val.gr = 256;
	params->others.bls_config.fixed_val.gb = 256;
	params->others.bls_config.fixed_val.b = 256;

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_BLS;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_BLS;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_BLS;
}

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
