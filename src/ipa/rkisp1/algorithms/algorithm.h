/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - RkISP1 control algorithm interface
 */

#pragma once

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::rkisp1 {

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
public:
	Algorithm()
		: disabled_(false), supportsRaw_(false)
	{
	}

	bool disabled_;
	bool supportsRaw_;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera */
