/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Matrix and related operations
 */

#include "libcamera/internal/matrix.h"

#include <libcamera/base/log.h>

/**
 * \file matrix.h
 * \brief Matrix class
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Matrix)

/**
 * \class Matrix
 * \brief Matrix class
 * \tparam T Type of numerical values to be stored in the matrix
 * \tparam Rows Number of rows in the matrix
 * \tparam Cols Number of columns in the matrix
 */

/**
 * \fn Matrix::Matrix()
 * \brief Construct a zero matrix
 */

/**
 * \fn Matrix::Matrix(const std::array<T, Rows * Cols> &data)
 * \brief Construct a matrix from supplied data
 * \param[in] data Data from which to construct a matrix
 *
 * \a data is a one-dimensional vector and will be turned into a matrix in
 * row-major order. The size of \a data must be equal to the product of the
 * number of rows and columns of the matrix (Rows x Cols).
 */

/**
 * \fn Matrix::identity()
 * \brief Construct an identity matrix
 */

/**
 * \fn Matrix::toString()
 * \brief Assemble and return a string describing the matrix
 * \return A string describing the matrix
 */

/**
 * \fn Matrix::data()
 * \brief Access the matrix data as a linear array
 *
 * Access the contents of the matrix as a one-dimensional linear array of
 * values in row-major order. The size of the array is equal to the product of
 * the number of rows and columns of the matrix (Rows x Cols).
 *
 * \return A span referencing the matrix data as a linear array
 */

/**
 * \fn Span<const T, Cols> Matrix::operator[](size_t i) const
 * \brief Index to a row in the matrix
 * \param[in] i Index of row to retrieve
 *
 * This operator[] returns a Span, which can then be indexed into again with
 * another operator[], allowing a convenient m[i][j] to access elements of the
 * matrix. Note that the lifetime of the Span returned by this first-level
 * operator[] is bound to that of the Matrix itself, so it is not recommended
 * to save the Span that is the result of this operator[].
 *
 * \return Row \a i from the matrix, as a Span
 */

/**
 * \fn Matrix::operator[](size_t i)
 * \copydoc Matrix::operator[](size_t i) const
 */

/**
 * \fn Matrix<T, Rows, Cols> &Matrix::operator*=(U d)
 * \brief Multiply the matrix by a scalar in-place
 * \tparam U Type of the numerical scalar value
 * \param d The scalar multiplier
 * \return Product of this matrix and scalar \a d
 */

/**
 * \fn Matrix::Matrix<U, Rows, Cols> operator*(T d, const Matrix<U, Rows, Cols> &m)
 * \brief Multiply the matrix by a scalar
 * \tparam T Type of the numerical scalar value
 * \tparam U Type of numerical values in the matrix
 * \tparam Rows Number of rows in the matrix
 * \tparam Cols Number of columns in the matrix
 * \param d The scalar multiplier
 * \param m The matrix
 * \return Product of scalar \a d and matrix \a m
 */

/**
 * \fn Matrix::Matrix<U, Rows, Cols> operator*(const Matrix<U, Rows, Cols> &m, T d)
 * \copydoc operator*(T d, const Matrix<U, Rows, Cols> &m)
 */

/**
 * \fn Matrix<T, R1, C2> operator*(const Matrix<T, R1, C1> &m1, const Matrix<T, R2, C2> &m2)
 * \brief Matrix multiplication
 * \tparam T Type of numerical values in the matrices
 * \tparam R1 Number of rows in the first matrix
 * \tparam C1 Number of columns in the first matrix
 * \tparam R2 Number of rows in the second matrix
 * \tparam C2 Number of columns in the second matrix
 * \param m1 Multiplicand matrix
 * \param m2 Multiplier matrix
 * \return Matrix product of matrices \a m1 and \a m2
 */

/**
 * \fn Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols> &m1, const Matrix<T, Rows, Cols> &m2)
 * \brief Matrix addition
 * \tparam T Type of numerical values in the matrices
 * \tparam Rows Number of rows in the matrices
 * \tparam Cols Number of columns in the matrices
 * \param m1 Summand matrix
 * \param m2 Summand matrix
 * \return Matrix sum of matrices \a m1 and \a m2
 */

#ifndef __DOXYGEN__
/*
 * The YAML data shall be a list of numerical values. Its size shall be equal
 * to the product of the number of rows and columns of the matrix (Rows x
 * Cols). The values shall be stored in row-major order.
 */
bool matrixValidateYaml(const YamlObject &obj, unsigned int size)
{
	if (!obj.isList())
		return false;

	if (obj.size() != size) {
		LOG(Matrix, Error)
			<< "Wrong number of values in matrix: expected "
			<< size << ", got " << obj.size();
		return false;
	}

	return true;
}
#endif /* __DOXYGEN__ */

} /* namespace libcamera */
