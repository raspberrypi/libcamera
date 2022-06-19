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

using Algorithm = libcamera::ipa::Algorithm<Module>;

} /* namespace ipa::rkisp1 */

} /* namespace libcamera */
