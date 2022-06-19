/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - IPU3 control algorithm interface
 */

#pragma once

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::ipu3 {

using Algorithm = libcamera::ipa::Algorithm<Module>;

} /* namespace ipa::ipu3 */

} /* namespace libcamera */
