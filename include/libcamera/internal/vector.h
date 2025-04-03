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
#include <functional>
#include <numeric>
#include <optional>
#include <ostream>
#include <type_traits>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Vector)

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

	constexpr explicit Vector(T scalar)
	{
		data_.fill(scalar);
	}

	constexpr Vector(const std::array<T, Rows> &data)
	{
		std::copy(data.begin(), data.end(), data_.begin());
	}

	constexpr Vector(const Span<const T, Rows> data)
	{
		std::copy(data.begin(), data.end(), data_.begin());
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

	constexpr Vector<T, Rows> operator-() const
	{
		Vector<T, Rows> ret;
		for (unsigned int i = 0; i < Rows; i++)
			ret[i] = -data_[i];
		return ret;
	}

	constexpr Vector operator+(const Vector &other) const
	{
		return apply(*this, other, std::plus<>{});
	}

	constexpr Vector operator+(T scalar) const
	{
		return apply(*this, scalar, std::plus<>{});
	}

	constexpr Vector operator-(const Vector &other) const
	{
		return apply(*this, other, std::minus<>{});
	}

	constexpr Vector operator-(T scalar) const
	{
		return apply(*this, scalar, std::minus<>{});
	}

	constexpr Vector operator*(const Vector &other) const
	{
		return apply(*this, other, std::multiplies<>{});
	}

	constexpr Vector operator*(T scalar) const
	{
		return apply(*this, scalar, std::multiplies<>{});
	}

	constexpr Vector operator/(const Vector &other) const
	{
		return apply(*this, other, std::divides<>{});
	}

	constexpr Vector operator/(T scalar) const
	{
		return apply(*this, scalar, std::divides<>{});
	}

	Vector &operator+=(const Vector &other)
	{
		return apply(other, [](T a, T b) { return a + b; });
	}

	Vector &operator+=(T scalar)
	{
		return apply(scalar, [](T a, T b) { return a + b; });
	}

	Vector &operator-=(const Vector &other)
	{
		return apply(other, [](T a, T b) { return a - b; });
	}

	Vector &operator-=(T scalar)
	{
		return apply(scalar, [](T a, T b) { return a - b; });
	}

	Vector &operator*=(const Vector &other)
	{
		return apply(other, [](T a, T b) { return a * b; });
	}

	Vector &operator*=(T scalar)
	{
		return apply(scalar, [](T a, T b) { return a * b; });
	}

	Vector &operator/=(const Vector &other)
	{
		return apply(other, [](T a, T b) { return a / b; });
	}

	Vector &operator/=(T scalar)
	{
		return apply(scalar, [](T a, T b) { return a / b; });
	}

	constexpr Vector min(const Vector &other) const
	{
		return apply(*this, other, [](T a, T b) { return std::min(a, b); });
	}

	constexpr Vector min(T scalar) const
	{
		return apply(*this, scalar, [](T a, T b) { return std::min(a, b); });
	}

	constexpr Vector max(const Vector &other) const
	{
		return apply(*this, other, [](T a, T b) { return std::max(a, b); });
	}

	constexpr Vector max(T scalar) const
	{
		return apply(*this, scalar, [](T a, T b) -> T { return std::max(a, b); });
	}

	constexpr T dot(const Vector<T, Rows> &other) const
	{
		T ret = 0;
		for (unsigned int i = 0; i < Rows; i++)
			ret += data_[i] * other[i];
		return ret;
	}

#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 1>>
#endif /* __DOXYGEN__ */
	constexpr const T &x() const { return data_[0]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 2>>
#endif /* __DOXYGEN__ */
	constexpr const T &y() const { return data_[1]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 3>>
#endif /* __DOXYGEN__ */
	constexpr const T &z() const { return data_[2]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 1>>
#endif /* __DOXYGEN__ */
	constexpr T &x() { return data_[0]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 2>>
#endif /* __DOXYGEN__ */
	constexpr T &y() { return data_[1]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 3>>
#endif /* __DOXYGEN__ */
	constexpr T &z() { return data_[2]; }

#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 1>>
#endif /* __DOXYGEN__ */
	constexpr const T &r() const { return data_[0]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 2>>
#endif /* __DOXYGEN__ */
	constexpr const T &g() const { return data_[1]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 3>>
#endif /* __DOXYGEN__ */
	constexpr const T &b() const { return data_[2]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 1>>
#endif /* __DOXYGEN__ */
	constexpr T &r() { return data_[0]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 2>>
#endif /* __DOXYGEN__ */
	constexpr T &g() { return data_[1]; }
#ifndef __DOXYGEN__
	template<bool Dependent = false, typename = std::enable_if_t<Dependent || Rows >= 3>>
#endif /* __DOXYGEN__ */
	constexpr T &b() { return data_[2]; }

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

	template<typename R = T>
	constexpr R sum() const
	{
		return std::accumulate(data_.begin(), data_.end(), R{});
	}

private:
	template<class BinaryOp>
	static constexpr Vector apply(const Vector &lhs, const Vector &rhs, BinaryOp op)
	{
		Vector result;
		std::transform(lhs.data_.begin(), lhs.data_.end(),
			       rhs.data_.begin(), result.data_.begin(),
			       op);

		return result;
	}

	template<class BinaryOp>
	static constexpr Vector apply(const Vector &lhs, T rhs, BinaryOp op)
	{
		Vector result;
		std::transform(lhs.data_.begin(), lhs.data_.end(),
			       result.data_.begin(),
			       [&op, rhs](T v) { return op(v, rhs); });

		return result;
	}

	template<class BinaryOp>
	Vector &apply(const Vector &other, BinaryOp op)
	{
		auto itOther = other.data_.begin();
		std::for_each(data_.begin(), data_.end(),
			      [&op, &itOther](T &v) { v = op(v, *itOther++); });

		return *this;
	}

	template<class BinaryOp>
	Vector &apply(T scalar, BinaryOp op)
	{
		std::for_each(data_.begin(), data_.end(),
			      [&op, scalar](T &v) { v = op(v, scalar); });

		return *this;
	}

	std::array<T, Rows> data_;
};

template<typename T>
using RGB = Vector<T, 3>;

template<typename T, typename U, unsigned int Rows, unsigned int Cols>
Vector<std::common_type_t<T, U>, Rows> operator*(const Matrix<T, Rows, Cols> &m, const Vector<U, Cols> &v)
{
	Vector<std::common_type_t<T, U>, Rows> result;

	for (unsigned int i = 0; i < Rows; i++) {
		std::common_type_t<T, U> sum = 0;
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

#ifndef __DOXYGEN__
template<typename T, unsigned int Rows>
std::ostream &operator<<(std::ostream &out, const Vector<T, Rows> &v)
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
struct YamlObject::Getter<Vector<T, Rows>> {
	std::optional<Vector<T, Rows>> get(const YamlObject &obj) const
	{
		if (!vectorValidateYaml(obj, Rows))
			return std::nullopt;

		Vector<T, Rows> vector;

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
