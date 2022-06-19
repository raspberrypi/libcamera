/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Ideas On Board
 *
 * module.h - IPU3 IPA Module
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/ipa/ipu3_ipa_interface.h>

#include <libipa/module.h>

#include "ipa_context.h"

namespace libcamera {

namespace ipa::ipu3 {

using Module = ipa::Module<IPAContext, IPAFrameContext, IPAConfigInfo,
			   ipu3_uapi_params, ipu3_uapi_stats_3a>;

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/
