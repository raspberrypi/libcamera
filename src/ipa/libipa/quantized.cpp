/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas On Board Oy
 *
 * Helper class to manage conversions between floating point types and quantized
 * storage and representation of those values.
 */

#include "quantized.h"

/**
 * \file quantized.h
 * \brief Quantized storage and Quantizer representations
 */

namespace libcamera {

namespace ipa {

/**
 * \struct libcamera::ipa::Quantized
 * \brief Wrapper that stores a value in both quantized and floating-point form
 * \tparam Traits The traits class defining the quantization behaviour
 *
 * The Quantized struct template provides a thin wrapper around a quantized
 * representation of a floating-point value. It uses a traits type \a Traits
 * to define the conversion policy between the floating-point domain and the
 * quantized integer domain.
 *
 * Each Quantized instance maintains two synchronized members:
 *  - the quantized integer representation, and
 *  - the corresponding floating-point value.
 *
 * The traits type defines:
 *  - the integer storage type used for quantization,
 *  - the static conversion functions \c fromFloat() and \c toFloat(), and
 *  - optional metadata such as value ranges.
 *
 * Quantized provides convenient constructors and assignment operators from
 * either representation, as well as comparison and string formatting utilities.
 */

/**
 * \typedef Quantized::TraitsType
 * \brief The traits policy type defining the quantization behaviour
 *
 * Exposes the associated traits type used by this Quantized instance.
 * This allows external code to refer to constants or metadata defined in
 * the traits, such as \c TraitsType::min or \c TraitsType::max.
 */

/**
 * \typedef Quantized::QuantizedType
 * \brief The integer type used for the quantized representation
 *
 * This alias corresponds to \c TraitsType::QuantizedType, as defined by
 * the traits class.
 */

/**
 * \fn Quantized::Quantized(float x)
 * \brief Construct a Quantized value from a floating-point number
 * \param[in] x The floating-point value to be quantized
 *
 * Converts the floating-point input \a x to its quantized integer
 * representation using the associated traits policy, and initializes
 * both the quantized and floating-point members.
 */

/**
 * \fn Quantized::Quantized(QuantizedType x)
 * \brief Construct a Quantized value from an existing quantized integer
 * \param[in] x The quantized integer value
 *
 * Converts the quantized integer \a x to its corresponding floating-point
 * value using the traits policy, and initializes both internal members.
 */

/**
 * \fn Quantized::operator=(float x)
 * \brief Assign a floating-point value to the Quantized object
 * \param[in] x The floating-point value to assign
 * \return A reference to the updated Quantized object
 *
 * Converts the floating-point value \a x to its quantized integer
 * representation using the traits policy and updates both members.
 */

/**
 * \fn Quantized::operator=(QuantizedType x)
 * \brief Assign a quantized integer value to the Quantized object
 * \param[in] x The quantized integer value to assign
 * \return A reference to the updated Quantized object
 *
 * Converts the quantized integer \a x to its corresponding floating-point
 * value using the traits policy and updates both members.
 */

/**
 * \fn Quantized::value() const noexcept
 * \brief Retrieve the floating-point representation
 * \return The floating-point value corresponding to the quantized value
 */

/**
 * \fn Quantized::quantized() const noexcept
 * \brief Retrieve the quantized integer representation
 * \return The quantized integer value
 */

/**
 * \fn Quantized::operator==(const Quantized &other) const noexcept
 * \brief Compare two Quantized objects for equality
 * \param[in] other The other Quantized object to compare against
 * \return True if both objects have the same quantized integer value
 */

/**
 * \fn Quantized::operator!=(const Quantized &other) const noexcept
 * \brief Compare two Quantized objects for inequality
 * \param[in] other The other Quantized object to compare against
 * \return True if the quantized integer values differ
 */

/**
 * \fn std::ostream &Quantized::operator<<(std::ostream &out, const Quantized<Traits> &q)
 * \brief Insert a text representation of a Quantized into an output stream
 * \param[in] out The output stream
 * \param[in] q The Quantized
 * \return The output stream \a out
 */

} /* namespace ipa */

} /* namespace libcamera */
