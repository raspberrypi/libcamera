/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * libipa miscellaneous colour helpers
 */

#include "colours.h"

#include <algorithm>
#include <cmath>

namespace libcamera {

namespace ipa {

/**
 * \file colours.h
 * \brief Functions to reduce code duplication between IPA modules
 */

/**
 * \brief Estimate luminance from RGB values following ITU-R BT.601
 * \param[in] rgb The RGB value
 *
 * This function estimates a luminance value from a triplet of Red, Green and
 * Blue values, following the formula defined by ITU-R Recommendation BT.601-7
 * which can be found at https://www.itu.int/rec/R-REC-BT.601
 *
 * \return The estimated luminance value
 */
double rec601LuminanceFromRGB(const RGB<double> &rgb)
{
	return (rgb.r() * .299) + (rgb.g() * .587) + (rgb.b() * .114);
}

/**
 * \brief Estimate correlated colour temperature from RGB color space input
 * \param[in] rgb The RGB value
 *
 * This function estimates the correlated color temperature RGB color space
 * input. In physics and color science, the Planckian locus or black body locus
 * is the path or locus that the color of an incandescent black body would take
 * in a particular chromaticity space as the black body temperature changes.
 *
 * If a narrow range of color temperatures is considered (those encapsulating
 * daylight being the most practical case) one can approximate the Planckian
 * locus in order to calculate the CCT in terms of chromaticity coordinates.
 *
 * More detailed information can be found in:
 * https://en.wikipedia.org/wiki/Color_temperature#Approximation
 *
 * \return The estimated color temperature
 */
uint32_t estimateCCT(const RGB<double> &rgb)
{
	/* Convert the RGB values to CIE tristimulus values (XYZ) */
	double X = (-0.14282) * rgb.r() + (1.54924) * rgb.g() + (-0.95641) * rgb.b();
	double Y = (-0.32466) * rgb.r() + (1.57837) * rgb.g() + (-0.73191) * rgb.b();
	double Z = (-0.68202) * rgb.r() + (0.77073) * rgb.g() + (0.56332) * rgb.b();

	/* Calculate the normalized chromaticity values */
	double x = X / (X + Y + Z);
	double y = Y / (X + Y + Z);

	/* Calculate CCT */
	double n = (x - 0.3320) / (0.1858 - y);
	return 449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33;
}

} /* namespace ipa */

} /* namespace libcamera */
