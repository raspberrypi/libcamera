/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Vector and related operations
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include "libcamera/internal/yaml_parser.h"

#include "matrix.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Vector)

namespace ipa {

#ifndef __DOXYGEN__
template<typename T, unsigned int Rows,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, unsigned int Rows>
#endif /* __DOXYGEN__ */
class Vector
{
public:
	constexpr Vector() = default;

	constexpr Vector(const std::array<T, Rows> &data)
	{
		for (unsigned int i = 0; i < Rows; i++)
			data_[i] = data[i];
	}

	const T &operator[](size_t i) const
	{
		ASSERT(i < data_.size());
		return data_[i];
	}

	T &operator[](size_t i)
	{
		ASSERT(i < data_.size());
		return data_[i];
	}

#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 1>>
#endif /* __DOXYGEN__ */
	constexpr T x() const
	{
		return data_[0];
	}

#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 2>>
#endif /* __DOXYGEN__ */
	constexpr T y() const
	{
		return data_[1];
	}

#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 3>>
#endif /* __DOXYGEN__ */
	constexpr T z() const
	{
		return data_[2];
	}

	constexpr Vector<T, Rows> operator-() const
	{
		Vector<T, Rows> ret;
		for (unsigned int i = 0; i < Rows; i++)
			ret[i] = -data_[i];
		return ret;
	}

	constexpr Vector<T, Rows> operator-(const Vector<T, Rows> &other) const
	{
		Vector<T, Rows> ret;
		for (unsigned int i = 0; i < Rows; i++)
			ret[i] = data_[i] - other[i];
		return ret;
	}

	constexpr Vector<T, Rows> operator+(const Vector<T, Rows> &other) const
	{
		Vector<T, Rows> ret;
		for (unsigned int i = 0; i < Rows; i++)
			ret[i] = data_[i] + other[i];
		return ret;
	}

	constexpr T operator*(const Vector<T, Rows> &other) const
	{
		T ret = 0;
		for (unsigned int i = 0; i < Rows; i++)
			ret += data_[i] * other[i];
		return ret;
	}

	constexpr Vector<T, Rows> operator*(T factor) const
	{
		Vector<T, Rows> ret;
		for (unsigned int i = 0; i < Rows; i++)
			ret[i] = data_[i] * factor;
		return ret;
	}

	constexpr Vector<T, Rows> operator/(T factor) const
	{
		Vector<T, Rows> ret;
		for (unsigned int i = 0; i < Rows; i++)
			ret[i] = data_[i] / factor;
		return ret;
	}

	constexpr double length2() const
	{
		double ret = 0;
		for (unsigned int i = 0; i < Rows; i++)
			ret += data_[i] * data_[i];
		return ret;
	}

	constexpr double length() const
	{
		return std::sqrt(length2());
	}

private:
	std::array<T, Rows> data_;
};

template<typename T, unsigned int Rows, unsigned int Cols>
Vector<T, Rows> operator*(const Matrix<T, Rows, Cols> &m, const Vector<T, Cols> &v)
{
	Vector<T, Rows> result;

	for (unsigned int i = 0; i < Rows; i++) {
		T sum = 0;
		for (unsigned int j = 0; j < Cols; j++)
			sum += m[i][j] * v[j];
		result[i] = sum;
	}

	return result;
}

template<typename T, unsigned int Rows>
bool operator==(const Vector<T, Rows> &lhs, const Vector<T, Rows> &rhs)
{
	for (unsigned int i = 0; i < Rows; i++) {
		if (lhs[i] != rhs[i])
			return false;
	}

	return true;
}

template<typename T, unsigned int Rows>
bool operator!=(const Vector<T, Rows> &lhs, const Vector<T, Rows> &rhs)
{
	return !(lhs == rhs);
}

#ifndef __DOXYGEN__
bool vectorValidateYaml(const YamlObject &obj, unsigned int size);
#endif /* __DOXYGEN__ */

} /* namespace ipa */

#ifndef __DOXYGEN__
template<typename T, unsigned int Rows>
std::ostream &operator<<(std::ostream &out, const ipa::Vector<T, Rows> &v)
{
	out << "Vector { ";
	for (unsigned int i = 0; i < Rows; i++) {
		out << v[i];
		out << ((i + 1 < Rows) ? ", " : " ");
	}
	out << " }";

	return out;
}

template<typename T, unsigned int Rows>
struct YamlObject::Getter<ipa::Vector<T, Rows>> {
	std::optional<ipa::Vector<T, Rows>> get(const YamlObject &obj) const
	{
		if (!ipa::vectorValidateYaml(obj, Rows))
			return std::nullopt;

		ipa::Vector<T, Rows> vector;

		unsigned int i = 0;
		for (const YamlObject &entry : obj.asList()) {
			const auto value = entry.get<T>();
			if (!value)
				return std::nullopt;
			vector[i++] = *value;
		}

		return vector;
	}
};
#endif /* __DOXYGEN__ */

} /* namespace libcamera */
