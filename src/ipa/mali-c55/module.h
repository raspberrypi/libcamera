/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Mali-C55 IPA Module
 */

#pragma once

#include <linux/mali-c55-config.h>

#include <libcamera/ipa/mali-c55_ipa_interface.h>

#include <libipa/module.h>

#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::mali_c55 {

using Module = ipa::Module<IPAContext, IPAFrameContext, IPACameraSensorInfo,
			   MaliC55Params, mali_c55_stats_buffer>;

} /* namespace ipa::mali_c55 */

} /* namespace libcamera*/
