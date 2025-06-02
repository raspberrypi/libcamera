/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Vector and related operations
 */

#include "libcamera/internal/vector.h"

#include <libcamera/base/log.h>

/**
 * \file vector.h
 * \brief Vector class
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Vector)

/**
 * \class Vector
 * \brief Vector class
 * \tparam T Type of numerical values to be stored in the vector
 * \tparam Rows Number of dimension of the vector (= number of elements)
 */

/**
 * \fn Vector::Vector()
 * \brief Construct an uninitialized vector
 */

/**
 * \fn Vector::Vector(T scalar)
 * \brief Construct a vector filled with a \a scalar value
 * \param[in] scalar The scalar value
 */

/**
 * \fn Vector::Vector(const std::array<T, Rows> &data)
 * \brief Construct vector from supplied data
 * \param data Data from which to construct a vector
 *
 * The size of \a data must be equal to the dimension size Rows of the vector.
 */

/**
 * \fn Vector::Vector(const Span<const T, Rows> data)
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
 * \fn Vector::operator-() const
 * \brief Negate a Vector by negating both all of its coordinates
 * \return The negated vector
 */

/**
 * \fn Vector::operator+(Vector const &other) const
 * \brief Calculate the sum of this vector and \a other element-wise
 * \param[in] other The other vector
 * \return The element-wise sum of this vector and \a other
 */

/**
 * \fn Vector::operator+(T scalar) const
 * \brief Calculate the sum of this vector and \a scalar element-wise
 * \param[in] scalar The scalar
 * \return The element-wise sum of this vector and \a other
 */

/**
 * \fn Vector::operator-(Vector const &other) const
 * \brief Calculate the difference of this vector and \a other element-wise
 * \param[in] other The other vector
 * \return The element-wise subtraction of \a other from this vector
 */

/**
 * \fn Vector::operator-(T scalar) const
 * \brief Calculate the difference of this vector and \a scalar element-wise
 * \param[in] scalar The scalar
 * \return The element-wise subtraction of \a scalar from this vector
 */

/**
 * \fn Vector::operator*(const Vector &other) const
 * \brief Calculate the product of this vector and \a other element-wise
 * \param[in] other The other vector
 * \return The element-wise product of this vector and \a other
 */

/**
 * \fn Vector::operator*(T scalar) const
 * \brief Calculate the product of this vector and \a scalar element-wise
 * \param[in] scalar The scalar
 * \return The element-wise product of this vector and \a scalar
 */

/**
 * \fn Vector::operator/(const Vector &other) const
 * \brief Calculate the quotient of this vector and \a other element-wise
 * \param[in] other The other vector
 * \return The element-wise division of this vector by \a other
 */

/**
 * \fn Vector::operator/(T scalar) const
 * \brief Calculate the quotient of this vector and \a scalar element-wise
 * \param[in] scalar The scalar
 * \return The element-wise division of this vector by \a scalar
 */

/**
 * \fn Vector::operator+=(Vector const &other)
 * \brief Add \a other element-wise to this vector
 * \param[in] other The other vector
 * \return This vector
 */

/**
 * \fn Vector::operator+=(T scalar)
 * \brief Add \a scalar element-wise to this vector
 * \param[in] scalar The scalar
 * \return This vector
 */

/**
 * \fn Vector::operator-=(Vector const &other)
 * \brief Subtract \a other element-wise from this vector
 * \param[in] other The other vector
 * \return This vector
 */

/**
 * \fn Vector::operator-=(T scalar)
 * \brief Subtract \a scalar element-wise from this vector
 * \param[in] scalar The scalar
 * \return This vector
 */

/**
 * \fn Vector::operator*=(const Vector &other)
 * \brief Multiply this vector by \a other element-wise
 * \param[in] other The other vector
 * \return This vector
 */

/**
 * \fn Vector::operator*=(T scalar)
 * \brief Multiply this vector by \a scalar element-wise
 * \param[in] scalar The scalar
 * \return This vector
 */

/**
 * \fn Vector::operator/=(const Vector &other)
 * \brief Divide this vector by \a other element-wise
 * \param[in] other The other vector
 * \return This vector
 */

/**
 * \fn Vector::operator/=(T scalar)
 * \brief Divide this vector by \a scalar element-wise
 * \param[in] scalar The scalar
 * \return This vector
 */

/**
 * \fn Vector::min(const Vector &other) const
 * \brief Calculate the minimum of this vector and \a other element-wise
 * \param[in] other The other vector
 * \return The element-wise minimum of this vector and \a other
 */

/**
 * \fn Vector::min(T scalar) const
 * \brief Calculate the minimum of this vector and \a scalar element-wise
 * \param[in] scalar The scalar
 * \return The element-wise minimum of this vector and \a scalar
 */

/**
 * \fn Vector::max(const Vector &other) const
 * \brief Calculate the maximum of this vector and \a other element-wise
 * \param[in] other The other vector
 * \return The element-wise maximum of this vector and \a other
 */

/**
 * \fn Vector::max(T scalar) const
 * \brief Calculate the maximum of this vector and \a scalar element-wise
 * \param[in] scalar The scalar
 * \return The element-wise maximum of this vector and \a scalar
 */

/**
 * \fn Vector::dot(const Vector<T, Rows> &other) const
 * \brief Compute the dot product
 * \param[in] other The other vector
 * \return The dot product of the two vectors
 */

/**
 * \fn constexpr T &Vector::x()
 * \brief Convenience function to access the first element of the vector
 * \return The first element of the vector
 */

/**
 * \fn constexpr T &Vector::y()
 * \brief Convenience function to access the second element of the vector
 * \return The second element of the vector
 */

/**
 * \fn constexpr T &Vector::z()
 * \brief Convenience function to access the third element of the vector
 * \return The third element of the vector
 */

/**
 * \fn constexpr const T &Vector::x() const
 * \copydoc Vector::x()
 */

/**
 * \fn constexpr const T &Vector::y() const
 * \copydoc Vector::y()
 */

/**
 * \fn constexpr const T &Vector::z() const
 * \copydoc Vector::z()
 */

/**
 * \fn constexpr T &Vector::r()
 * \brief Convenience function to access the first element of the vector
 * \return The first element of the vector
 */

/**
 * \fn constexpr T &Vector::g()
 * \brief Convenience function to access the second element of the vector
 * \return The second element of the vector
 */

/**
 * \fn constexpr T &Vector::b()
 * \brief Convenience function to access the third element of the vector
 * \return The third element of the vector
 */

/**
 * \fn constexpr const T &Vector::r() const
 * \copydoc Vector::r()
 */

/**
 * \fn constexpr const T &Vector::g() const
 * \copydoc Vector::g()
 */

/**
 * \fn constexpr const T &Vector::b() const
 * \copydoc Vector::b()
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
 * \fn Vector::sum() const
 * \brief Calculate the sum of all the vector elements
 * \tparam R The type of the sum
 *
 * The type R of the sum defaults to the type T of the elements, but can be set
 * explicitly to use a different type in case the type T would risk
 * overflowing.
 *
 * \return The sum of all the vector elements
 */

/**
 * \fn operator*(const Matrix<T, Rows, Cols> &m, const Vector<U, Cols> &v)
 * \brief Multiply a matrix by a vector
 * \tparam T Numerical type of the contents of the matrix
 * \tparam U Numerical type of the contents of the vector
 * \tparam Rows The number of rows in the matrix
 * \tparam Cols The number of columns in the matrix (= rows in the vector)
 * \param m The matrix
 * \param v The vector
 * \return Product of matrix \a m and vector \a v
 */

/**
 * \typedef RGB
 * \brief A Vector of 3 elements representing an RGB pixel value
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

} /* namespace libcamera */
