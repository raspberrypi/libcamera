/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Fixed / floating point conversions
 */

#include "fixedpoint.h"

/**
 * \file fixedpoint.h
 */

namespace libcamera {

namespace ipa {

/**
 * \struct libcamera::ipa::FixedPointQTraits
 * \brief Traits type implementing fixed-point quantization conversions
 *
 * The FixedPointQTraits structure defines a policy for mapping floating-point
 * values to and from fixed-point 2's complement integer representations. It is
 * parameterised by the number of integer bits \a I, fractional bits \a F, and
 * the integral storage type \a T. The traits are used with Quantized<Traits> to
 * create a quantized type that stores both the fixed-point representation and
 * the corresponding floating-point value.
 *
 * The signedness of the type is determined by the signedness of \a T. For
 * signed types, the number of integer bits in \a I includes the sign bit.
 *
 * Storage is determined by the total number of bits \a (I + F) and is
 * automatically selected, but the internal storage type is always an unsigned
 * integer to guarantee against sign extension when storing quantized values
 * in registers.
 *
 * The trait exposes compile-time constants describing the bit layout, limits,
 * and scaling factors used in the fixed-point representation.
 *
 * \tparam I Number of integer bits
 * \tparam F Number of fractional bits
 * \tparam T Integral type used to store the quantized value
 */

/**
 * \typedef FixedPointQTraits::QuantizedType
 * \brief The integral storage type used for the fixed-point representation
 */

/**
 * \var FixedPointQTraits::qMin
 * \brief Minimum representable quantized integer value
 *
 * This corresponds to the most negative value for signed formats or zero for
 * unsigned formats.
 */

/**
 * \var FixedPointQTraits::qMax
 * \brief Maximum representable quantized integer value
 */

/**
 * \var FixedPointQTraits::min
 * \brief Minimum representable floating-point value corresponding to qMin
 */

/**
 * \var FixedPointQTraits::max
 * \brief Maximum representable floating-point value corresponding to qMax
 */

/**
 * \fn FixedPointQTraits::fromFloat(float v)
 * \brief Convert a floating-point value to a fixed-point integer
 * \param[in] v The floating-point value to be converted
 * \return The quantized fixed-point integer representation
 *
 * The conversion first clamps the floating-point input \a v to the range [min,
 * max] and then rounds it to the nearest fixed-point value according to the
 * scaling factor defined by the number of fractional bits F.
 */

/**
 * \fn FixedPointQTraits::toFloat(QuantizedType q)
 * \brief Convert a fixed-point integer to a floating-point value
 * \param[in] q The fixed-point integer value to be converted
 * \return The corresponding floating-point value
 *
 * The conversion sign-extends the integer value if required and divides by the
 * scaling factor defined by the number of fractional bits F.
 */

/**
 * \typedef Q
 * \brief Define a signed fixed-point quantized type with automatic storage width
 * \tparam I The number of integer bits
 * \tparam F The number of fractional bits
 *
 * This alias defines a signed fixed-point quantized type using the
 * \ref FixedPointQTraits trait and a suitable signed integer storage type
 * automatically selected based on the total number of bits \a (I + F).
 */

/**
 * \typedef UQ
 * \brief Define an unsigned fixed-point quantized type with automatic storage width
 * \tparam I The number of integer bits
 * \tparam F The number of fractional bits
 *
 * This alias defines an unsigned fixed-point quantized type using the
 * \ref FixedPointQTraits trait and a suitable unsigned integer storage type
 * automatically selected based on the total number of bits \a (I + F).
 */

} /* namespace ipa */

} /* namespace libcamera */
