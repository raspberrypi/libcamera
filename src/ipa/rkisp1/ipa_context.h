/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipa_context.h - RkISP1 IPA Context
 *
 */

#pragma once

#include <linux/rkisp1-config.h>

namespace libcamera {

namespace ipa::rkisp1 {

struct IPASessionConfiguration {
	struct {
		rkisp1_cif_isp_version revision;
	} hw;
};

struct IPAFrameContext {
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAFrameContext frameContext;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
