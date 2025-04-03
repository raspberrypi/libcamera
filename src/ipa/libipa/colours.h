/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * libipa miscellaneous colour helpers
 */

#pragma once

#include <stdint.h>

#include "libcamera/internal/vector.h"

namespace libcamera {

namespace ipa {

double rec601LuminanceFromRGB(const RGB<double> &rgb);
uint32_t estimateCCT(const RGB<double> &rgb);

} /* namespace ipa */

} /* namespace libcamera */
