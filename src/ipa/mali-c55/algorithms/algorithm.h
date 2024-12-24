/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * algorithm.h - Mali-C55 control algorithm interface
 */

#pragma once

#include <linux/mali-c55-config.h>

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::mali_c55 {

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
};

union mali_c55_params_block {
	struct mali_c55_params_block_header *header;
	struct mali_c55_params_sensor_off_preshading *sensor_offs;
	struct mali_c55_params_aexp_hist *aexp_hist;
	struct mali_c55_params_aexp_weights *aexp_weights;
	struct mali_c55_params_digital_gain *digital_gain;
	struct mali_c55_params_awb_gains *awb_gains;
	struct mali_c55_params_awb_config *awb_config;
	struct mali_c55_params_mesh_shading_config *shading_config;
	struct mali_c55_params_mesh_shading_selection *shading_selection;
	__u8 *data;
};

} /* namespace ipa::mali_c55 */

} /* namespace libcamera */
