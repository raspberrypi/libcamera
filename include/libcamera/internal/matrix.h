/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Matrix and related operations
 */
#pragma once

#include <algorithm>
#include <sstream>
#include <type_traits>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Matrix)

#ifndef __DOXYGEN__
template<typename T>
bool matrixInvert(Span<const T> dataIn, Span<T> dataOut, unsigned int dim,
		  Span<T> scratchBuffer, Span<unsigned int> swapBuffer);
#endif /* __DOXYGEN__ */

template<typename T, unsigned int Rows, unsigned int Cols>
class Matrix
{
	static_assert(std::is_arithmetic_v<T>, "Matrix type must be arithmetic");

public:
	constexpr Matrix()
	{
	}

	Matrix(const std::array<T, Rows * Cols> &data)
	{
		std::copy(data.begin(), data.end(), data_.begin());
	}

	Matrix(const Span<const T, Rows * Cols> data)
	{
		std::copy(data.begin(), data.end(), data_.begin());
	}

	static constexpr Matrix identity()
	{
		Matrix ret;
		for (size_t i = 0; i < std::min(Rows, Cols); i++)
			ret[i][i] = static_cast<T>(1);
		return ret;
	}

	~Matrix() = default;

	const std::string toString() const
	{
		std::stringstream out;

		out << "Matrix { ";
		for (unsigned int i = 0; i < Rows; i++) {
			out << "[ ";
			for (unsigned int j = 0; j < Cols; j++) {
				out << (*this)[i][j];
				out << ((j + 1 < Cols) ? ", " : " ");
			}
			out << ((i + 1 < Rows) ? "], " : "]");
		}
		out << " }";

		return out.str();
	}

	constexpr Span<const T, Rows * Cols> data() const { return data_; }

	constexpr Span<const T, Cols> operator[](size_t i) const
	{
		return Span<const T, Cols>{ &data_.data()[i * Cols], Cols };
	}

	constexpr Span<T, Cols> operator[](size_t i)
	{
		return Span<T, Cols>{ &data_.data()[i * Cols], Cols };
	}

#ifndef __DOXYGEN__
	template<typename U, std::enable_if_t<std::is_arithmetic_v<U>>>
#else
	template<typename U>
#endif /* __DOXYGEN__ */
	Matrix<T, Rows, Cols> &operator*=(U d)
	{
		for (unsigned int i = 0; i < Rows * Cols; i++)
			data_[i] *= d;
		return *this;
	}

	Matrix<T, Rows, Cols> inverse(bool *ok = nullptr) const
	{
		static_assert(Rows == Cols, "Matrix must be square");

		Matrix<T, Rows, Cols> inverse;
		std::array<T, Rows * Cols * 2> scratchBuffer;
		std::array<unsigned int, Rows> swapBuffer;
		bool res = matrixInvert(Span<const T>(data_),
					Span<T>(inverse.data_),
					Rows,
					Span<T>(scratchBuffer),
					Span<unsigned int>(swapBuffer));
		if (ok)
			*ok = res;
		return inverse;
	}

private:
	/*
	 * \todo The initializer is only necessary for the constructor to be
	 * constexpr in C++17. Remove the initializer as soon as we are on
	 * C++20.
	 */
	std::array<T, Rows * Cols> data_ = {};
};

#ifndef __DOXYGEN__
template<typename T, typename U, unsigned int Rows, unsigned int Cols,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, typename U, unsigned int Rows, unsigned int Cols>
#endif /* __DOXYGEN__ */
Matrix<U, Rows, Cols> operator*(T d, const Matrix<U, Rows, Cols> &m)
{
	Matrix<U, Rows, Cols> result;

	for (unsigned int i = 0; i < Rows; i++) {
		for (unsigned int j = 0; j < Cols; j++)
			result[i][j] = d * m[i][j];
	}

	return result;
}

#ifndef __DOXYGEN__
template<typename T, typename U, unsigned int Rows, unsigned int Cols,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, typename U, unsigned int Rows, unsigned int Cols>
#endif /* __DOXYGEN__ */
Matrix<U, Rows, Cols> operator*(const Matrix<U, Rows, Cols> &m, T d)
{
	return d * m;
}

template<typename T1, unsigned int R1, unsigned int C1, typename T2, unsigned int R2, unsigned int C2>
constexpr Matrix<std::common_type_t<T1, T2>, R1, C2> operator*(const Matrix<T1, R1, C1> &m1,
							       const Matrix<T2, R2, C2> &m2)
{
	static_assert(C1 == R2, "Matrix dimensions must match for multiplication");
	Matrix<std::common_type_t<T1, T2>, R1, C2> result;

	for (unsigned int i = 0; i < R1; i++) {
		for (unsigned int j = 0; j < C2; j++) {
			std::common_type_t<T1, T2> sum = 0;

			for (unsigned int k = 0; k < C1; k++)
				sum += m1[i][k] * m2[k][j];

			result[i][j] = sum;
		}
	}

	return result;
}

template<typename T, unsigned int Rows, unsigned int Cols>
constexpr Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols> &m1, const Matrix<T, Rows, Cols> &m2)
{
	Matrix<T, Rows, Cols> result;

	for (unsigned int i = 0; i < Rows; i++) {
		for (unsigned int j = 0; j < Cols; j++)
			result[i][j] = m1[i][j] + m2[i][j];
	}

	return result;
}

#ifndef __DOXYGEN__
bool matrixValidateYaml(const YamlObject &obj, unsigned int size);
#endif /* __DOXYGEN__ */

#ifndef __DOXYGEN__
template<typename T, unsigned int Rows, unsigned int Cols>
std::ostream &operator<<(std::ostream &out, const Matrix<T, Rows, Cols> &m)
{
	out << m.toString();
	return out;
}

template<typename T, unsigned int Rows, unsigned int Cols>
struct YamlObject::Getter<Matrix<T, Rows, Cols>> {
	std::optional<Matrix<T, Rows, Cols>> get(const YamlObject &obj) const
	{
		if (!matrixValidateYaml(obj, Rows * Cols))
			return std::nullopt;

		Matrix<T, Rows, Cols> matrix;
		T *data = &matrix[0][0];

		unsigned int i = 0;
		for (const YamlObject &entry : obj.asList()) {
			const auto value = entry.get<T>();
			if (!value)
				return std::nullopt;

			data[i++] = *value;
		}

		return matrix;
	}
};
#endif /* __DOXYGEN__ */

} /* namespace libcamera */
