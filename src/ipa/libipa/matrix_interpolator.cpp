/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class for interpolating maps of matrices
 */
#include "matrix_interpolator.h"

#include <algorithm>
#include <string>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "matrix.h"

/**
 * \file matrix_interpolator.h
 * \brief Helper class for interpolating maps of matrices
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(MatrixInterpolator)

namespace ipa {

/**
 * \class MatrixInterpolator
 * \brief Class for storing, retrieving, and interpolating matrices
 * \tparam T Type of numerical values to be stored in the matrices
 * \tparam R Number of rows in the matrices
 * \tparam C Number of columns in the matrices
 *
 * The main use case is to pass a map from color temperatures to corresponding
 * matrices (eg. color correction), and then requesting a matrix for a specific
 * color temperature. This class will abstract away the interpolation portion.
 */

/**
 * \fn MatrixInterpolator::MatrixInterpolator(const std::map<unsigned int, Matrix<T, R, C>> &matrices)
 * \brief Construct a matrix interpolator from a map of matrices
 * \param matrices Map from which to construct the matrix interpolator
 */

/**
 * \fn MatrixInterpolator::reset()
 * \brief Reset the matrix interpolator content to a single identity matrix
 */

/**
 * \fn int MatrixInterpolator<T, R, C>::readYaml()
 * \brief Initialize an MatrixInterpolator instance from yaml
 * \tparam T Type of data stored in the matrices
 * \tparam R Number of rows of the matrices
 * \tparam C Number of columns of the matrices
 * \param[in] yaml The yaml object that contains the map of unsigned integers to matrices
 * \param[in] key_name The name of the key in the yaml object
 * \param[in] matrix_name The name of the matrix in the yaml object
 *
 * The yaml object is expected to be a list of maps. Each map has two or more
 * pairs: one of \a key_name to the key value (usually color temperature), and
 * one or more of \a matrix_name to the matrix. This is a bit difficult to
 * explain, so here is an example (in python, as it is easier to parse than
 * yaml):
 *       [
 *               {
 *                   'ct': 2860,
 *                   'ccm': [ 2.12089, -0.52461, -0.59629,
 *                           -0.85342,  2.80445, -0.95103,
 *                           -0.26897, -1.14788,  2.41685 ],
 *                   'offsets': [ 0, 0, 0 ]
 *               },
 *
 *               {
 *                   'ct': 2960,
 *                   'ccm': [ 2.26962, -0.54174, -0.72789,
 *                           -0.77008,  2.60271, -0.83262,
 *                           -0.26036, -1.51254,  2.77289 ],
 *                   'offsets': [ 0, 0, 0 ]
 *               },
 *
 *               {
 *                   'ct': 3603,
 *                   'ccm': [ 2.18644, -0.66148, -0.52496,
 *                           -0.77828,  2.69474, -0.91645,
 *                           -0.25239, -0.83059,  2.08298 ],
 *                   'offsets': [ 0, 0, 0 ]
 *               },
 *       ]
 *
 * In this case, \a key_name would be 'ct', and \a matrix_name can be either
 * 'ccm' or 'offsets'. This way multiple matrix interpolators can be defined in
 * one set of color temperature ranges in the tuning file, and they can be
 * retrieved separately with the \a matrix_name parameter.
 *
 * \return Zero on success, negative error code otherwise
 */

/**
 * \fn Matrix<T, R, C> MatrixInterpolator<T, R, C>::get(unsigned int key)
 * \brief Retrieve a matrix from the list of matrices, interpolating if necessary
 * \param[in] key The unsigned integer key of the matrix to retrieve
 * \return The matrix corresponding to the color temperature
 */

} /* namespace ipa */

} /* namespace libcamera */
