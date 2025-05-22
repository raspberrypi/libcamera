/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Matrix and related operations
 */

#include "libcamera/internal/matrix.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <numeric>
#include <vector>

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
 * \fn Matrix::Matrix(const Span<const T, Rows * Cols> data)
 * \brief Construct a matrix from supplied data
 * \param[in] data Data from which to construct a matrix
 *
 * \a data is a one-dimensional Span and will be turned into a matrix in
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
 * \fn Matrix::inverse(bool *ok) const
 * \param[out] ok Indicate if the matrix was successfully inverted
 * \brief Compute the inverse of the matrix
 *
 * This function computes the inverse of the matrix. It is only implemented for
 * matrices of float and double types. If \a ok is provided it will be set to a
 * boolean value to indicate of the inversion was successful. This can be used
 * to check if the matrix is singular, in which case the function will return
 * an identity matrix.
 *
 * \return The inverse of the matrix
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
 * \fn operator*(const Matrix<T1, R1, C1> &m1, const Matrix<T2, R2, C2> &m2)
 * \brief Matrix multiplication
 * \tparam T1 Type of numerical values in the first matrix
 * \tparam R1 Number of rows in the first matrix
 * \tparam C1 Number of columns in the first matrix
 * \tparam T2 Type of numerical values in the secont matrix
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
template<typename T>
bool matrixInvert(Span<const T> dataIn, Span<T> dataOut, unsigned int dim,
		  Span<T> scratchBuffer, Span<unsigned int> swapBuffer)
{
	/*
	 * Convenience class to access matrix data, providing a row-major (i,j)
	 * element accessor through the call operator, and the ability to swap
	 * rows without modifying the backing storage.
	 */
	class MatrixAccessor
	{
	public:
		MatrixAccessor(Span<T> data, Span<unsigned int> swapBuffer, unsigned int rows, unsigned int cols)
			: data_(data), swap_(swapBuffer), rows_(rows), cols_(cols)
		{
			ASSERT(swap_.size() == rows);
			std::iota(swap_.begin(), swap_.end(), T{ 0 });
		}

		T &operator()(unsigned int row, unsigned int col)
		{
			assert(row < rows_ && col < cols_);
			return data_[index(row, col)];
		}

		void swap(unsigned int a, unsigned int b)
		{
			assert(a < rows_ && a < cols_);
			std::swap(swap_[a], swap_[b]);
		}

	private:
		unsigned int index(unsigned int row, unsigned int col) const
		{
			return swap_[row] * cols_ + col;
		}

		Span<T> data_;
		Span<unsigned int> swap_;
		unsigned int rows_;
		unsigned int cols_;
	};

	/*
	 * Matrix inversion using Gaussian elimination.
	 *
	 * Start by augmenting the original matrix with an identiy matrix of
	 * the same size.
	 */
	ASSERT(scratchBuffer.size() == dim * dim * 2);
	MatrixAccessor matrix(scratchBuffer, swapBuffer, dim, dim * 2);

	for (unsigned int i = 0; i < dim; ++i) {
		for (unsigned int j = 0; j < dim; ++j) {
			matrix(i, j) = dataIn[i * dim + j];
			matrix(i, j + dim) = T{ 0 };
		}
		matrix(i, i + dim) = T{ 1 };
	}

	/* Start by triangularizing the input . */
	for (unsigned int pivot = 0; pivot < dim; ++pivot) {
		/*
		 * Locate the next pivot. To improve numerical stability, use
		 * the row with the largest value in the pivot's column.
		 */
		unsigned int row = pivot;
		T maxValue{ 0 };

		for (unsigned int i = pivot; i < dim; ++i) {
			T value = std::abs(matrix(i, pivot));
			if (maxValue < value) {
				maxValue = value;
				row = i;
			}
		}

		/*
		 * If no pivot is found in the column, the matrix is not
		 * invertible. Return an identity matrix.
		 */
		if (maxValue == 0) {
			std::fill(dataOut.begin(), dataOut.end(), T{ 0 });
			for (unsigned int i = 0; i < dim; ++i)
				dataOut[i * dim + i] = T{ 1 };
			return false;
		}

		/* Swap rows to bring the pivot in the right location. */
		matrix.swap(pivot, row);

		/* Process all rows below the pivot to zero the pivot column. */
		const T pivotValue = matrix(pivot, pivot);

		for (unsigned int i = pivot + 1; i < dim; ++i) {
			const T factor = matrix(i, pivot) / pivotValue;

			/*
			 * We know the element in the pivot column will be 0,
			 * hardcode it instead of computing it.
			 */
			matrix(i, pivot) = T{ 0 };

			for (unsigned int j = pivot + 1; j < dim * 2; ++j)
				matrix(i, j) -= matrix(pivot, j) * factor;
		}
	}

	/*
	 * Then diagonalize the input, walking the diagonal backwards. There's
	 * no need to update the input matrix, as all the values we would write
	 * in the top-right triangle aren't used in further calculations (and
	 * would all by definition be zero).
	 */
	for (unsigned int pivot = dim - 1; pivot > 0; --pivot) {
		const T pivotValue = matrix(pivot, pivot);

		for (unsigned int i = 0; i < pivot; ++i) {
			const T factor = matrix(i, pivot) / pivotValue;

			for (unsigned int j = dim; j < dim * 2; ++j)
				matrix(i, j) -= matrix(pivot, j) * factor;
		}
	}

	/*
	 * Finally, normalize the diagonal and store the result in the output
	 * data.
	 */
	for (unsigned int i = 0; i < dim; ++i) {
		const T factor = matrix(i, i);

		for (unsigned int j = 0; j < dim; ++j)
			dataOut[i * dim + j] = matrix(i, j + dim) / factor;
	}

	return true;
}

template bool matrixInvert<float>(Span<const float> dataIn, Span<float> dataOut,
				  unsigned int dim, Span<float> scratchBuffer,
				  Span<unsigned int> swapBuffer);
template bool matrixInvert<double>(Span<const double> data, Span<double> dataOut,
				   unsigned int dim, Span<double> scratchBuffer,
				   Span<unsigned int> swapBuffer);

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
