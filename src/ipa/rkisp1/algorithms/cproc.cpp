/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * cproc.cpp - RkISP1 Color Processing control
 */

#include "cproc.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

/**
 * \file cproc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class ColorProcessing
 * \brief RkISP1 Color Processing control
 *
 * The ColorProcessing algorithm is responsible for applying brightness,
 * contrast and saturation corrections. The values are directly provided
 * through requests by the corresponding controls.
 */

LOG_DEFINE_CATEGORY(RkISP1CProc)

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void ColorProcessing::queueRequest(IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   const ControlList &controls)
{
	auto &cproc = context.frameContext.cproc;

	const auto &brightness = controls.get(controls::Brightness);
	if (brightness) {
		cproc.brightness = std::clamp<int>(std::lround(*brightness * 128), -128, 127);
		cproc.updateParams = true;

		LOG(RkISP1CProc, Debug) << "Set brightness to " << *brightness;
	}

	const auto &contrast = controls.get(controls::Contrast);
	if (contrast) {
		cproc.contrast = std::clamp<int>(std::lround(*contrast * 128), 0, 255);
		cproc.updateParams = true;

		LOG(RkISP1CProc, Debug) << "Set contrast to " << *contrast;
	}

	const auto saturation = controls.get(controls::Saturation);
	if (saturation) {
		cproc.saturation = std::clamp<int>(std::lround(*saturation * 128), 0, 255);
		cproc.updateParams = true;

		LOG(RkISP1CProc, Debug) << "Set saturation to " << *saturation;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void ColorProcessing::prepare(IPAContext &context,
			      rkisp1_params_cfg *params)
{
	auto &cproc = context.frameContext.cproc;

	/* Check if the algorithm configuration has been updated. */
	if (!cproc.updateParams)
		return;

	cproc.updateParams = false;

	params->others.cproc_config.brightness = cproc.brightness;
	params->others.cproc_config.contrast = cproc.contrast;
	params->others.cproc_config.sat = cproc.saturation;

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_CPROC;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_CPROC;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_CPROC;
}

REGISTER_IPA_ALGORITHM(ColorProcessing, "ColorProcessing")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
