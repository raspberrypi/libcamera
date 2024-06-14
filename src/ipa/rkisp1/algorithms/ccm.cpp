/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 Color Correction Matrix control algorithm
 */

#include "ccm.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/yaml_parser.h"

#include "../utils.h"
#include "libipa/matrix_interpolator.h"

/**
 * \file ccm.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class Ccm
 * \brief A color correction matrix algorithm
 */

LOG_DEFINE_CATEGORY(RkISP1Ccm)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Ccm::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	int ret = ccm_.readYaml(tuningData["ccms"], "ct", "ccm");
	if (ret < 0) {
		LOG(RkISP1Ccm, Warning)
			<< "Failed to parse 'ccm' "
			<< "parameter from tuning file; falling back to unit matrix";
		ccm_.reset();
	}

	ret = offsets_.readYaml(tuningData["ccms"], "ct", "offsets");
	if (ret < 0) {
		LOG(RkISP1Ccm, Warning)
			<< "Failed to parse 'offsets' "
			<< "parameter from tuning file; falling back to zero offsets";
		/*
		 * MatrixInterpolator::reset() resets to identity matrices
		 * while here we need zero matrices so we need to construct it
		 * ourselves.
		 */
		Matrix<int16_t, 3, 1> m({ 0, 0, 0 });
		std::map<unsigned int, Matrix<int16_t, 3, 1>> matrices = { { 0, m } };
		offsets_ = MatrixInterpolator<int16_t, 3, 1>(matrices);
	}

	return 0;
}

void Ccm::setParameters(rkisp1_params_cfg *params,
			const Matrix<float, 3, 3> &matrix,
			const Matrix<int16_t, 3, 1> &offsets)
{
	struct rkisp1_cif_isp_ctk_config &config = params->others.ctk_config;

	/*
	 * 4 bit integer and 7 bit fractional, ranging from -8 (0x400) to
	 * +7.992 (0x3ff)
	 */
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			config.coeff[i][j] =
				utils::floatingToFixedPoint<4, 7, uint16_t, double>(matrix[i][j]);
	}

	for (unsigned int i = 0; i < 3; i++)
		config.ct_offset[i] = offsets[i][0] & 0xfff;

	LOG(RkISP1Ccm, Debug) << "Setting matrix " << matrix;
	LOG(RkISP1Ccm, Debug) << "Setting offsets " << offsets;

	params->module_en_update |= RKISP1_CIF_ISP_MODULE_CTK;
	params->module_ens |= RKISP1_CIF_ISP_MODULE_CTK;
	params->module_cfg_update |= RKISP1_CIF_ISP_MODULE_CTK;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext,
		  rkisp1_params_cfg *params)
{
	uint32_t ct = context.activeState.awb.temperatureK;

	/*
	 * \todo The colour temperature will likely be noisy, add filtering to
	 * avoid updating the CCM matrix all the time.
	 */
	if (frame > 0 && ct == ct_)
		return;

	ct_ = ct;
	Matrix<float, 3, 3> ccm = ccm_.get(ct);
	Matrix<int16_t, 3, 1> offsets = offsets_.get(ct);

	frameContext.ccm.ccm = ccm;

	setParameters(params, ccm, offsets);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const rkisp1_stat_buffer *stats,
		  ControlList &metadata)
{
	float m[9];
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			m[i] = frameContext.ccm.ccm[i][j];
	}
	metadata.set(controls::ColourCorrectionMatrix, m);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
