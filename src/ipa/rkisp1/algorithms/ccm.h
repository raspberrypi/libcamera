/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * RkISP1 Color Correction Matrix control algorithm
 */

#pragma once

#include <linux/rkisp1-config.h>

#include "libipa/matrix.h"
#include "libipa/matrix_interpolator.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Ccm : public Algorithm
{
public:
	Ccm() {}
	~Ccm() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     rkisp1_params_cfg *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const rkisp1_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	void parseYaml(const YamlObject &tuningData);
	void setParameters(rkisp1_params_cfg *params,
			   const Matrix<float, 3, 3> &matrix,
			   const Matrix<int16_t, 3, 1> &offsets);

	unsigned int ct_;
	MatrixInterpolator<float, 3, 3> ccm_;
	MatrixInterpolator<int16_t, 3, 1> offsets_;
};

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
