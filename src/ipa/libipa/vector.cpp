/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Vector and related operations
 */

#include "vector.h"

#include <libcamera/base/log.h>

/**
 * \file vector.h
 * \brief Vector class
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Vector)

namespace ipa {

/**
 * \class Vector
 * \brief Vector class
 * \tparam T Type of numerical values to be stored in the vector
 * \tparam Rows Number of dimension of the vector (= number of elements)
 */

/**
 * \fn Vector::Vector()
 * \brief Construct a zero vector
 */

/**
 * \fn Vector::Vector(const std::array<T, Rows> &data)
 * \brief Construct vector from supplied data
 * \param data Data from which to construct a vector
 *
 * The size of \a data must be equal to the dimension size Rows of the vector.
 */

/**
 * \fn T Vector::operator[](size_t i) const
 * \brief Index to an element in the vector
 * \param i Index of element to retrieve
 * \return Element at index \a i from the vector
 */

/**
 * \fn T &Vector::operator[](size_t i)
 * \copydoc Vector::operator[](size_t i) const
 */

/**
 * \fn Vector::x()
 * \brief Convenience function to access the first element of the vector
 * \return The first element of the vector
 */

/**
 * \fn Vector::y()
 * \brief Convenience function to access the second element of the vector
 * \return The second element of the vector
 */

/**
 * \fn Vector::z()
 * \brief Convenience function to access the third element of the vector
 * \return The third element of the vector
 */

/**
 * \fn Vector::operator-() const
 * \brief Negate a Vector by negating both all of its coordinates
 * \return The negated vector
 */

/**
 * \fn Vector::operator-(Vector const &other) const
 * \brief Subtract one vector from another
 * \param[in] other The other vector
 * \return The difference of \a other from this vector
 */

/**
 * \fn Vector::operator+()
 * \brief Add two vectors together
 * \param[in] other The other vector
 * \return The sum of the two vectors
 */

/**
 * \fn Vector::operator*(const Vector<T, Rows> &other) const
 * \brief Compute the dot product
 * \param[in] other The other vector
 * \return The dot product of the two vectors
 */

/**
 * \fn Vector::operator*(T factor) const
 * \brief Multiply the vector by a scalar
 * \param[in] factor The factor
 * \return The vector multiplied by \a factor
 */

/**
 * \fn Vector::operator/()
 * \brief Divide the vector by a scalar
 * \param[in] factor The factor
 * \return The vector divided by \a factor
 */

/**
 * \fn Vector::length2()
 * \brief Get the squared length of the vector
 * \return The squared length of the vector
 */

/**
 * \fn Vector::length()
 * \brief Get the length of the vector
 * \return The length of the vector
 */

/**
 * \fn Vector<T, Rows> operator*(const Matrix<T, Rows, Cols> &m, const Vector<T, Cols> &v)
 * \brief Multiply a matrix by a vector
 * \tparam T Numerical type of the contents of the matrix and vector
 * \tparam Rows The number of rows in the matrix
 * \tparam Cols The number of columns in the matrix (= rows in the vector)
 * \param m The matrix
 * \param v The vector
 * \return Product of matrix \a m and vector \a v
 */

/**
 * \fn bool operator==(const Vector<T, Rows> &lhs, const Vector<T, Rows> &rhs)
 * \brief Compare vectors for equality
 * \return True if the two vectors are equal, false otherwise
 */

/**
 * \fn bool operator!=(const Vector<T, Rows> &lhs, const Vector<T, Rows> &rhs)
 * \brief Compare vectors for inequality
 * \return True if the two vectors are not equal, false otherwise
 */

#ifndef __DOXYGEN__
bool vectorValidateYaml(const YamlObject &obj, unsigned int size)
{
	if (!obj.isList())
		return false;

	if (obj.size() != size) {
		LOG(Vector, Error)
			<< "Wrong number of values in YAML vector: expected "
			<< size << ", got " << obj.size();
		return false;
	}

	return true;
}
#endif /* __DOXYGEN__ */

} /* namespace ipa */

} /* namespace libcamera */
