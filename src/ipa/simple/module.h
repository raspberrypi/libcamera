/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Red Hat, Inc.
 *
 * Software ISP IPA Module
 */

#pragma once

#include <libcamera/controls.h>

#include <libcamera/ipa/soft_ipa_interface.h>

#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

#include <libipa/module.h>

#include "ipa_context.h"

namespace libcamera {

namespace ipa::soft {

using Module = ipa::Module<IPAContext, IPAFrameContext, IPAConfigInfo,
			   DebayerParams, SwIspStats>;

} /* namespace ipa::soft */

} /* namespace libcamera */
