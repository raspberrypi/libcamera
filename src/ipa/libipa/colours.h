/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * libipa miscellaneous colour helpers
 */

#pragma once

#include <stdint.h>

namespace libcamera {

namespace ipa {

double rec601LuminanceFromRGB(double r, double g, double b);
uint32_t estimateCCT(double red, double green, double blue);

} /* namespace ipa */

} /* namespace libcamera */
