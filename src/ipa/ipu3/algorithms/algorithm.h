/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - IPU3 control algorithm interface
 */

#pragma once

#include <libcamera/ipa/ipu3_ipa_interface.h>

#include <libipa/algorithm.h>

#include "ipa_context.h"

namespace libcamera {

namespace ipa::ipu3 {

using Algorithm = libcamera::ipa::Algorithm<IPAContext, IPAConfigInfo, ipu3_uapi_params, ipu3_uapi_stats_3a>;

} /* namespace ipa::ipu3 */

} /* namespace libcamera */
