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
	static const Vector<double, 3> rgb2y{{
		0.299, 0.587, 0.114
	}};

	return rgb.dot(rgb2y);
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
	/*
	 * Convert the RGB values to CIE tristimulus values (XYZ) and divide by
	 * the sum of X, Y and Z to calculate the CIE xy chromaticity.
	 */
	static const Matrix<double, 3, 3> rgb2xyz({
		-0.14282, 1.54924, -0.95641,
		-0.32466, 1.57837, -0.73191,
		-0.68202, 0.77073,  0.56332
	});

	Vector<double, 3> xyz = rgb2xyz * rgb;
	xyz /= xyz.sum();

	/* Calculate CCT */
	double n = (xyz.x() - 0.3320) / (0.1858 - xyz.y());
	return 449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33;
}

} /* namespace ipa */

} /* namespace libcamera */
