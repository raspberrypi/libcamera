/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - RkISP1 control algorithm interface
 */

#pragma once

#include <linux/rkisp1-config.h>

#include <libcamera/ipa/rkisp1_ipa_interface.h>

#include <libipa/algorithm.h>

#include "ipa_context.h"

namespace libcamera {

namespace ipa::rkisp1 {

using Algorithm = libcamera::ipa::Algorithm<IPAContext, IPACameraSensorInfo, rkisp1_params_cfg, rkisp1_stat_buffer>;

} /* namespace ipa::rkisp1 */

} /* namespace libcamera */
