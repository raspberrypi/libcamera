/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * dpf.cpp - RkISP1 Denoise Pre-Filter control
 */

#include "dpf.h"

#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "linux/rkisp1-config.h"

/**
 * \file dpf.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class Dpf
 * \brief RkISP1 Denoise Pre-Filter control
 *
 * The denoise pre-filter algorithm is a bilateral filter which combines a
 * range filter and a domain filter. The denoise pre-filter is applied before
 * demosaicing.
 */

LOG_DEFINE_CATEGORY(RkISP1Dpf)

Dpf::Dpf()
	: config_({}), strengthConfig_({})
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Dpf::init([[maybe_unused]] IPAContext &context,
	      const YamlObject &tuningData)
{
	std::vector<uint8_t> values;

	/*
	 * The domain kernel is configured with a 9x9 kernel for the green
	 * pixels, and a 13x9 or 9x9 kernel for red and blue pixels.
	 */
	const YamlObject &dFObject = tuningData["DomainFilter"];

	/*
	 * For the green component, we have the 9x9 kernel specified
	 * as 6 coefficients:
	 *    Y
	 *    ^
	 *  4 | 6   5   4   5   6
	 *  3 |   5   3   3   5
	 *  2 | 5   3   2   3   5
	 *  1 |   3   1   1   3
	 *  0 - 4   2   0   2   4
	 * -1 |   3   1   1   3
	 * -2 | 5   3   2   3   5
	 * -3 |   5   3   3   5
	 * -4 | 6   5   4   5   6
	 *    +---------|--------> X
	 *     -4....-1 0 1 2 3 4
	 */
	values = dFObject["g"].getList<uint8_t>().value_or(std::vector<uint8_t>{});
	if (values.size() != RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS) {
		LOG(RkISP1Dpf, Error)
			<< "Invalid 'DomainFilter:g': expected "
			<< RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS
			<< " elements, got " << values.size();
		return -EINVAL;
	}

	std::copy_n(values.begin(), values.size(),
		    std::begin(config_.g_flt.spatial_coeff));

	config_.g_flt.gr_enable = true;
	config_.g_flt.gb_enable = true;

	/*
	 * For the red and blue components, we have the 13x9 kernel specified
	 * as 6 coefficients:
	 *
	 *    Y
	 *    ^
	 *  4 | 6   5   4   3   4   5   6
	 *    |
	 *  2 | 5   4   2   1   2   4   5
	 *    |
	 *  0 - 5   3   1   0   1   3   5
	 *    |
	 * -2 | 5   4   2   1   2   4   5
	 *    |
	 * -4 | 6   5   4   3   4   5   6
	 *    +-------------|------------> X
	 *     -6  -4  -2   0   2   4   6
	 *
	 * For a 9x9 kernel, columns -6 and 6 are dropped, so coefficient
	 * number 6 is not used.
	 */
	values = dFObject["rb"].getList<uint8_t>().value_or(std::vector<uint8_t>{});
	if (values.size() != RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS &&
	    values.size() != RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS - 1) {
		LOG(RkISP1Dpf, Error)
			<< "Invalid 'DomainFilter:rb': expected "
			<< RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS - 1
			<< " or " << RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS
			<< " elements, got " << values.size();
		return -EINVAL;
	}

	config_.rb_flt.fltsize = values.size() == RKISP1_CIF_ISP_DPF_MAX_SPATIAL_COEFFS
			       ? RKISP1_CIF_ISP_DPF_RB_FILTERSIZE_13x9
			       : RKISP1_CIF_ISP_DPF_RB_FILTERSIZE_9x9;

	std::copy_n(values.begin(), values.size(),
		    std::begin(config_.rb_flt.spatial_coeff));

	config_.rb_flt.r_enable = true;
	config_.rb_flt.b_enable = true;

	/*
	 * The range kernel is configured with a noise level lookup table (NLL)
	 * which stores a piecewise linear function that characterizes the
	 * sensor noise profile as a noise level function curve (NLF).
	 */
	const YamlObject &rFObject = tuningData["NoiseLevelFunction"];

	std::vector<uint16_t> nllValues;
	nllValues = rFObject["coeff"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (nllValues.size() != RKISP1_CIF_ISP_DPF_MAX_NLF_COEFFS) {
		LOG(RkISP1Dpf, Error)
			<< "Invalid 'RangeFilter:coeff': expected "
			<< RKISP1_CIF_ISP_DPF_MAX_NLF_COEFFS
			<< " elements, got " << nllValues.size();
		return -EINVAL;
	}

	std::copy_n(nllValues.begin(), nllValues.size(),
		    std::begin(config_.nll.coeff));

	std::string scaleMode = rFObject["scale-mode"].get<std::string>("");
	if (scaleMode == "linear") {
		config_.nll.scale_mode = RKISP1_CIF_ISP_NLL_SCALE_LINEAR;
	} else if (scaleMode == "logarithmic") {
		config_.nll.scale_mode = RKISP1_CIF_ISP_NLL_SCALE_LOGARITHMIC;
	} else {
		LOG(RkISP1Dpf, Error)
			<< "Invalid 'RangeFilter:scale-mode': expected "
			<< "'linear' or 'logarithmic' value, got "
			<< scaleMode;
		return -EINVAL;
	}

	const YamlObject &fSObject = tuningData["FilterStrength"];

	strengthConfig_.r = fSObject["r"].get<uint16_t>(64);
	strengthConfig_.g = fSObject["g"].get<uint16_t>(64);
	strengthConfig_.b = fSObject["b"].get<uint16_t>(64);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Dpf::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &dpf = context.activeState.dpf;
	bool update = false;

	const auto &denoise = controls.get(controls::draft::NoiseReductionMode);
	if (denoise) {
		LOG(RkISP1Dpf, Debug) << "Set denoise to " << *denoise;

		switch (*denoise) {
		case controls::draft::NoiseReductionModeOff:
			if (dpf.denoise) {
				dpf.denoise = false;
				update = true;
			}
			break;
		case controls::draft::NoiseReductionModeMinimal:
		case controls::draft::NoiseReductionModeHighQuality:
		case controls::draft::NoiseReductionModeFast:
			if (!dpf.denoise) {
				dpf.denoise = true;
				update = true;
			}
			break;
		default:
			LOG(RkISP1Dpf, Error)
				<< "Unsupported denoise value "
				<< *denoise;
			break;
		}
	}

	frameContext.dpf.denoise = dpf.denoise;
	frameContext.dpf.update = update;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Dpf::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, rkisp1_params_cfg *params)
{
	if (frame == 0) {
		params->others.dpf_config = config_;
		params->others.dpf_strength_config = strengthConfig_;

		const auto &awb = context.configuration.awb;
		const auto &lsc = context.configuration.lsc;
		auto &mode = params->others.dpf_config.gain.mode;

		/*
		 * The DPF needs to take into account the total amount of
		 * digital gain, which comes from the AWB and LSC modules. The
		 * DPF hardware can be programmed with a digital gain value
		 * manually, but can also use the gains supplied by the AWB and
		 * LSC modules automatically when they are enabled. Use that
		 * mode of operation as it simplifies control of the DPF.
		 */
		if (awb.enabled && lsc.enabled)
			mode = RKISP1_CIF_ISP_DPF_GAIN_USAGE_AWB_LSC_GAINS;
		else if (awb.enabled)
			mode = RKISP1_CIF_ISP_DPF_GAIN_USAGE_AWB_GAINS;
		else if (lsc.enabled)
			mode = RKISP1_CIF_ISP_DPF_GAIN_USAGE_LSC_GAINS;
		else
			mode = RKISP1_CIF_ISP_DPF_GAIN_USAGE_DISABLED;

		params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_DPF |
					     RKISP1_CIF_ISP_MODULE_DPF_STRENGTH;
	}

	if (frameContext.dpf.update) {
		params->module_en_update |= RKISP1_CIF_ISP_MODULE_DPF;
		if (frameContext.dpf.denoise)
			params->module_ens |= RKISP1_CIF_ISP_MODULE_DPF;
	}
}

REGISTER_IPA_ALGORITHM(Dpf, "Dpf")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
