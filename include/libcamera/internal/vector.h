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
#include "libcamera/internal/value_node.h"

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
		return apply(*this, std::plus<>{}, other);
	}

	constexpr Vector operator+(T scalar) const
	{
		return apply(*this, std::plus<>{}, scalar);
	}

	constexpr Vector operator-(const Vector &other) const
	{
		return apply(*this, std::minus<>{}, other);
	}

	constexpr Vector operator-(T scalar) const
	{
		return apply(*this, std::minus<>{}, scalar);
	}

	constexpr Vector operator*(const Vector &other) const
	{
		return apply(*this, std::multiplies<>{}, other);
	}

	constexpr Vector operator*(T scalar) const
	{
		return apply(*this, std::multiplies<>{}, scalar);
	}

	constexpr Vector operator/(const Vector &other) const
	{
		return apply(*this, std::divides<>{}, other);
	}

	constexpr Vector operator/(T scalar) const
	{
		return apply(*this, std::divides<>{}, scalar);
	}

	constexpr Vector operator>>(unsigned int shift) const
	{
		static_assert(std::is_integral_v<T>,
			      "Vector::operator>> requires an integer element type");
		return apply(*this, [](T a, unsigned int b) { return a >> b; }, shift);
	}

	Vector &operator+=(const Vector &other)
	{
		return apply(std::plus<>{}, other);
	}

	Vector &operator+=(T scalar)
	{
		return apply(std::plus<>{}, scalar);
	}

	Vector &operator-=(const Vector &other)
	{
		return apply(std::minus<>{}, other);
	}

	Vector &operator-=(T scalar)
	{
		return apply(std::minus<>{}, scalar);
	}

	Vector &operator*=(const Vector &other)
	{
		return apply(std::multiplies<>{}, other);
	}

	Vector &operator*=(T scalar)
	{
		return apply(std::multiplies<>{}, scalar);
	}

	Vector &operator/=(const Vector &other)
	{
		return apply(std::divides<>{}, other);
	}

	Vector &operator/=(T scalar)
	{
		return apply(std::divides<>{}, scalar);
	}

	Vector &operator>>=(unsigned int shift)
	{
		static_assert(std::is_integral_v<T>,
			      "Vector::operator>>= requires an integer element type");
		return apply([](T a, unsigned int b) { return a >> b; }, shift);
	}

	constexpr Vector min(const Vector &other) const
	{
		return apply(*this, [](T a, T b) { return std::min(a, b); }, other);
	}

	constexpr Vector min(T scalar) const
	{
		return apply(*this, [](T a, T b) { return std::min(a, b); }, scalar);
	}

	constexpr Vector max(const Vector &other) const
	{
		return apply(*this, [](T a, T b) { return std::max(a, b); }, other);
	}

	constexpr Vector max(T scalar) const
	{
		return apply(*this, [](T a, T b) -> T { return std::max(a, b); }, scalar);
	}

	constexpr Vector clamp(T low, T high) const
	{
		return apply(*this,
			     [](T v, T lo, T hi) -> T { return std::clamp(v, lo, hi); },
			     low, high);
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
	template<typename BinaryOp>
	static constexpr Vector apply(const Vector &lhs, BinaryOp op, const Vector &rhs)
	{
		Vector result;
		std::transform(lhs.data_.begin(), lhs.data_.end(),
			       rhs.data_.begin(), result.data_.begin(),
			       op);

		return result;
	}

	template<typename Op, typename... U>
	static constexpr Vector apply(const Vector &vector, Op op, U... scalar)
	{
		Vector result;
		std::transform(vector.data_.begin(), vector.data_.end(),
			       result.data_.begin(),
			       [&op, scalar...](T v) { return op(v, scalar...); });

		return result;
	}

	template<typename BinaryOp>
	Vector &apply(BinaryOp op, const Vector &other)
	{
		auto itOther = other.data_.begin();
		std::for_each(data_.begin(), data_.end(),
			      [&op, &itOther](T &v) { v = op(v, *itOther++); });

		return *this;
	}

	template<typename Op, typename... U>
	Vector &apply(Op op, U... scalar)
	{
		std::for_each(data_.begin(), data_.end(),
			      [&op, scalar...](T &v) { v = op(v, scalar...); });

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
bool vectorValidateValueNode(const ValueNode &obj, unsigned int size);

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
struct ValueNode::Accessor<Vector<T, Rows>> {
	std::optional<Vector<T, Rows>> get(const ValueNode &obj) const
	{
		if (!vectorValidateValueNode(obj, Rows))
			return std::nullopt;

		Vector<T, Rows> vector;

		unsigned int i = 0;
		for (const ValueNode &entry : obj.asList()) {
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
