/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Miscellaneous utility functions specific to rkisp1
 */

#include "utils.h"

/**
 * \file utils.h
 */

namespace libcamera {

namespace ipa::rkisp1::utils {

/**
 * \fn R floatingToFixedPoint(T number)
 * \brief Convert a floating point number to a fixed-point representation
 * \tparam I Bit width of the integer part of the fixed-point
 * \tparam F Bit width of the fractional part of the fixed-point
 * \tparam R Return type of the fixed-point representation
 * \tparam T Input type of the floating point representation
 * \param number The floating point number to convert to fixed point
 * \return The converted value
 */

/**
 * \fn R fixedToFloatingPoint(T number)
 * \brief Convert a fixed-point number to a floating point representation
 * \tparam I Bit width of the integer part of the fixed-point
 * \tparam F Bit width of the fractional part of the fixed-point
 * \tparam R Return type of the floating point representation
 * \tparam T Input type of the fixed-point representation
 * \param number The fixed point number to convert to floating point
 * \return The converted value
 */

} /* namespace ipa::rkisp1::utils */

} /* namespace libcamera */
