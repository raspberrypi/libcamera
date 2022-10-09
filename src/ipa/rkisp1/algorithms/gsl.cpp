/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * gsl.cpp - RkISP1 Gamma Sensor Linearization control
 */

#include "gsl.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "linux/rkisp1-config.h"

/**
 * \file gsl.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class GammaSensorLinearization
 * \brief RkISP1 Gamma Sensor Linearization control
 *
 * This algorithm linearizes the sensor output to compensate the sensor
 * non-linearities by applying piecewise linear functions to the red, green and
 * blue channels.
 *
 * The curves are specified in the tuning data and defined using 17 points.
 *
 * - The X coordinates are expressed using 16 intervals, with the first point
 *   at X coordinate 0. Each interval is expressed as a 2-bit value DX (from
 *   GAMMA_DX_1 to GAMMA_DX_16), stored in the RKISP1_CIF_ISP_GAMMA_DX_LO and
 *   RKISP1_CIF_ISP_GAMMA_DX_HI registers. The real interval is equal to
 *   \f$2^{dx+4}\f$. X coordinates are shared between the red, green and blue
 *   curves.
 *
 * - The Y coordinates are specified as 17 values separately for the
 *   red, green and blue channels, with a 12-bit resolution. Each value must be
 *   in the [-2048, 2047] range compared to the previous value.
 */

LOG_DEFINE_CATEGORY(RkISP1Gsl)

static constexpr unsigned int kDegammaXIntervals = 16;

GammaSensorLinearization::GammaSensorLinearization()
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int GammaSensorLinearization::init([[maybe_unused]] IPAContext &context,
				   const YamlObject &tuningData)
{
	std::vector<uint16_t> xIntervals =
		tuningData["x-intervals"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (xIntervals.size() != kDegammaXIntervals) {
		LOG(RkISP1Gsl, Error)
			<< "Invalid 'x' coordinates: expected "
			<< kDegammaXIntervals << " elements, got "
			<< xIntervals.size();

		return -EINVAL;
	}

	/* Compute gammaDx_ intervals from xIntervals values */
	gammaDx_[0] = 0;
	gammaDx_[1] = 0;
	for (unsigned int i = 0; i < kDegammaXIntervals; ++i)
		gammaDx_[i / 8] |= (xIntervals[i] & 0x07) << ((i % 8) * 4);

	const YamlObject &yObject = tuningData["y"];
	if (!yObject.isDictionary()) {
		LOG(RkISP1Gsl, Error)
			<< "Issue while parsing 'y' in tuning file: "
			<< "entry must be a dictionary";
		return -EINVAL;
	}

	curveYr_ = yObject["red"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (curveYr_.size() != RKISP1_CIF_ISP_DEGAMMA_CURVE_SIZE) {
		LOG(RkISP1Gsl, Error)
			<< "Invalid 'y:red' coordinates: expected "
			<< RKISP1_CIF_ISP_DEGAMMA_CURVE_SIZE
			<< " elements, got " << curveYr_.size();
		return -EINVAL;
	}

	curveYg_ = yObject["green"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (curveYg_.size() != RKISP1_CIF_ISP_DEGAMMA_CURVE_SIZE) {
		LOG(RkISP1Gsl, Error)
			<< "Invalid 'y:green' coordinates: expected "
			<< RKISP1_CIF_ISP_DEGAMMA_CURVE_SIZE
			<< " elements, got " << curveYg_.size();
		return -EINVAL;
	}

	curveYb_ = yObject["blue"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (curveYb_.size() != RKISP1_CIF_ISP_DEGAMMA_CURVE_SIZE) {
		LOG(RkISP1Gsl, Error)
			<< "Invalid 'y:blue' coordinates: expected "
			<< RKISP1_CIF_ISP_DEGAMMA_CURVE_SIZE
			<< " elements, got " << curveYb_.size();
		return -EINVAL;
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void GammaSensorLinearization::prepare([[maybe_unused]] IPAContext &context,
				       const uint32_t frame,
				       [[maybe_unused]] IPAFrameContext &frameContext,
				       rkisp1_params_cfg *params)
{
	if (frame > 0)
		return;

	params->others.sdg_config.xa_pnts.gamma_dx0 = gammaDx_[0];
	params->others.sdg_config.xa_pnts.gamma_dx1 = gammaDx_[1];

	std::copy(curveYr_.begin(), curveYr_.end(),
		  params->others.sdg_config.curve_r.gamma_y);
	std::copy(curveYg_.begin(), curveYg_.end(),
		  params->others.sdg_config.curve_g.gamma_y);
	std::copy(curveYb_.begin(), curveYb_.end(),
		  params->others.sdg_config.curve_b.gamma_y);

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_SDG;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_SDG;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_SDG;
}

REGISTER_IPA_ALGORITHM(GammaSensorLinearization, "GammaSensorLinearization")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
