/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Polynomial class to represent lens shading correction
 */

#include "lsc_polynomial.h"

#include <libcamera/base/log.h>

/**
 * \file lsc_polynomial.h
 * \brief LscPolynomial class
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(LscPolynomial)

namespace ipa {

/**
 * \class LscPolynomial
 * \brief Class for handling even polynomials used in lens shading correction
 *
 * Shading artifacts of camera lenses can be modeled using even radial
 * polynomials. This class implements a polynomial with 5 coefficients which
 * follows the definition of the FixVignetteRadial opcode in the Adobe DNG
 * specification.
 */

/**
 * \fn LscPolynomial::LscPolynomial(double cx = 0.0, double cy = 0.0, double k0 = 0.0,
		      double k1 = 0.0, double k2 = 0.0, double k3 = 0.0,
		      double k4 = 0.0)
 * \brief Construct a polynomial using the given coefficients
 * \param cx Center-x relative to the image in normalized coordinates (0..1)
 * \param cy Center-y relative to the image in normalized coordinates (0..1)
 * \param k0 Coefficient of the polynomial
 * \param k1 Coefficient of the polynomial
 * \param k2 Coefficient of the polynomial
 * \param k3 Coefficient of the polynomial
 * \param k4 Coefficient of the polynomial
 */

/**
 * \fn LscPolynomial::sampleAtNormalizedPixelPos(double x, double y)
 * \brief Sample the polynomial at the given normalized pixel position
 *
 * This functions samples the polynomial at the given pixel position divided by
 * the value returned by getM().
 *
 * \param x x position in normalized coordinates
 * \param y y position in normalized coordinates
 * \return The sampled value
 */

/**
 * \fn LscPolynomial::getM()
 * \brief Get the value m as described in the dng specification
 *
 * Returns m according to dng spec. m represents the Euclidean distance
 * (in pixels) from the optical center to the farthest pixel in the
 * image.
 *
 * \return The sampled value
 */

/**
 * \fn LscPolynomial::setReferenceImageSize(const Size &size)
 * \brief Set the reference image size
 *
 * Set the reference image size that is used for subsequent calls to getM() and
 * sampleAtNormalizedPixelPos()
 *
 * \param size The size of the reference image
 */

} // namespace ipa
} // namespace libcamera
