/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Helper class for interpolating objects
 */
#include "interpolator.h"

#include <algorithm>
#include <string>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "interpolator.h"

/**
 * \file interpolator.h
 * \brief Helper class for linear interpolating a set of objects
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Interpolator)

namespace ipa {

/**
 * \class Interpolator
 * \brief Class for storing, retrieving, and interpolating objects
 * \tparam T Type of objects stored in the interpolator
 *
 * The main use case is to pass a map from color temperatures to corresponding
 * objects (eg. matrices for color correction), and then requesting a
 * interpolated object for a specific color temperature. This class will
 * abstract away the interpolation portion.
 */

/**
 * \fn Interpolator::Interpolator()
 * \brief Construct an empty interpolator
 */

/**
 * \fn Interpolator::Interpolator(const std::map<unsigned int, T> &data)
 * \brief Construct an interpolator from a map of objects
 * \param data Map from which to construct the interpolator
 */

/**
 * \fn Interpolator::Interpolator(std::map<unsigned int, T> &&data)
 * \brief Construct an interpolator from a map of objects
 * \param data Map from which to construct the interpolator
 */

/**
 * \fn int Interpolator<T>::readYaml(const libcamera::YamlObject &yaml,
		                     const std::string &key_name,
		                     const std::string &value_name)
 * \brief Initialize an Interpolator instance from yaml
 * \tparam T Type of data stored in the interpolator
 * \param[in] yaml The yaml object that contains the map of unsigned integers to
 * objects
 * \param[in] key_name The name of the key in the yaml object
 * \param[in] value_name The name of the value in the yaml object
 *
 * The yaml object is expected to be a list of maps. Each map has two or more
 * pairs: one of \a key_name to the key value (usually color temperature), and
 * one or more of \a value_name to the object. This is a bit difficult to
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
 * In this case, \a key_name would be 'ct', and \a value_name can be either
 * 'ccm' or 'offsets'. This way multiple interpolators can be defined in
 * one set of color temperature ranges in the tuning file, and they can be
 * retrieved separately with the \a value_name parameter.
 *
 * \return Zero on success, negative error code otherwise
 */

/**
 * \fn void Interpolator<T>::setQuantization(const unsigned int q)
 * \brief Set the quantization value
 * \param[in] q The quantization value
 *
 * Sets the quantization value. When this is set, 'key' gets quantized to this
 * size, before doing the interpolation. This can help in reducing the number of
 * updates pushed to the hardware.
 *
 * Note that normally a threshold needs to be combined with quantization.
 * Otherwise a value that swings around the edge of the quantization step will
 * lead to constant updates.
 */

/**
 * \fn void Interpolator<T>::setData(std::map<unsigned int, T> &&data)
 * \brief Set the internal map
 *
 * Overwrites the internal map using move semantics.
 */

/**
 * \fn std::map<unsigned int, T> &Interpolator<T>::data() const
 * \brief Access the internal map
 *
 * \return The internal map
 */

/**
 * \fn const T& Interpolator<T>::getInterpolated()
 * \brief Retrieve an interpolated value for the given key
 * \param[in] key The unsigned integer key of the object to retrieve
 * \param[out] quantizedKey If provided, the key value after quantization
 * \return The object corresponding to the key. The object is cached internally,
 * so on successive calls with the same key (after quantization) interpolation
 * is not recalculated.
 */

/**
 * \fn void Interpolator<T>::interpolate(const T &a, const T &b, T &dest, double lambda)
 * \brief Interpolate between two instances of T
 * \param a The first value to interpolate
 * \param b The second value to interpolate
 * \param dest The destination for the interpolated value
 * \param lambda The interpolation factor (0..1)
 *
 * Interpolates between \a a and \a b according to \a lambda. It calculates
 * dest = a * (1.0 - lambda) + b * lambda;
 *
 * If T supports multiplication with double and addition, this function can be
 * used as is. For other types this function can be overwritten using partial
 * template specialization.
 */

} /* namespace ipa */

} /* namespace libcamera */
