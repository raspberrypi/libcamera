/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.h - Geometry-related classes
 */

#pragma once

#include <algorithm>
#include <ostream>
#include <string>

#include <libcamera/base/compiler.h>

namespace libcamera {

class Rectangle;

class Point
{
public:
	constexpr Point()
		: x(0), y(0)
	{
	}

	constexpr Point(int xpos, int ypos)
		: x(xpos), y(ypos)
	{
	}

	int x;
	int y;

	const std::string toString() const;

	constexpr Point operator-() const
	{
		return { -x, -y };
	}
};

bool operator==(const Point &lhs, const Point &rhs);
static inline bool operator!=(const Point &lhs, const Point &rhs)
{
	return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &out, const Point &p);

class Size
{
public:
	constexpr Size()
		: Size(0, 0)
	{
	}

	constexpr Size(unsigned int w, unsigned int h)
		: width(w), height(h)
	{
	}

	unsigned int width;
	unsigned int height;

	bool isNull() const { return !width && !height; }
	const std::string toString() const;

	Size &alignDownTo(unsigned int hAlignment, unsigned int vAlignment)
	{
		width = width / hAlignment * hAlignment;
		height = height / vAlignment * vAlignment;
		return *this;
	}

	Size &alignUpTo(unsigned int hAlignment, unsigned int vAlignment)
	{
		width = (width + hAlignment - 1) / hAlignment * hAlignment;
		height = (height + vAlignment - 1) / vAlignment * vAlignment;
		return *this;
	}

	Size &boundTo(const Size &bound)
	{
		width = std::min(width, bound.width);
		height = std::min(height, bound.height);
		return *this;
	}

	Size &expandTo(const Size &expand)
	{
		width = std::max(width, expand.width);
		height = std::max(height, expand.height);
		return *this;
	}

	Size &growBy(const Size &margins)
	{
		width += margins.width;
		height += margins.height;
		return *this;
	}

	Size &shrinkBy(const Size &margins)
	{
		width = width > margins.width ? width - margins.width : 0;
		height = height > margins.height ? height - margins.height : 0;
		return *this;
	}

	__nodiscard constexpr Size alignedDownTo(unsigned int hAlignment,
						 unsigned int vAlignment) const
	{
		return {
			width / hAlignment * hAlignment,
			height / vAlignment * vAlignment
		};
	}

	__nodiscard constexpr Size alignedUpTo(unsigned int hAlignment,
					       unsigned int vAlignment) const
	{
		return {
			(width + hAlignment - 1) / hAlignment * hAlignment,
			(height + vAlignment - 1) / vAlignment * vAlignment
		};
	}

	__nodiscard constexpr Size boundedTo(const Size &bound) const
	{
		return {
			std::min(width, bound.width),
			std::min(height, bound.height)
		};
	}

	__nodiscard constexpr Size expandedTo(const Size &expand) const
	{
		return {
			std::max(width, expand.width),
			std::max(height, expand.height)
		};
	}

	__nodiscard constexpr Size grownBy(const Size &margins) const
	{
		return {
			width + margins.width,
			height + margins.height
		};
	}

	__nodiscard constexpr Size shrunkBy(const Size &margins) const
	{
		return {
			width > margins.width ? width - margins.width : 0,
			height > margins.height ? height - margins.height : 0
		};
	}

	__nodiscard Size boundedToAspectRatio(const Size &ratio) const;
	__nodiscard Size expandedToAspectRatio(const Size &ratio) const;

	__nodiscard Rectangle centeredTo(const Point &center) const;

	Size operator*(float factor) const;
	Size operator/(float factor) const;

	Size &operator*=(float factor);
	Size &operator/=(float factor);
};

bool operator==(const Size &lhs, const Size &rhs);
bool operator<(const Size &lhs, const Size &rhs);

static inline bool operator!=(const Size &lhs, const Size &rhs)
{
	return !(lhs == rhs);
}

static inline bool operator<=(const Size &lhs, const Size &rhs)
{
	return lhs < rhs || lhs == rhs;
}

static inline bool operator>(const Size &lhs, const Size &rhs)
{
	return !(lhs <= rhs);
}

static inline bool operator>=(const Size &lhs, const Size &rhs)
{
	return !(lhs < rhs);
}

std::ostream &operator<<(std::ostream &out, const Size &s);

class SizeRange
{
public:
	SizeRange()
		: hStep(0), vStep(0)
	{
	}

	SizeRange(const Size &size)
		: min(size), max(size), hStep(1), vStep(1)
	{
	}

	SizeRange(const Size &minSize, const Size &maxSize)
		: min(minSize), max(maxSize), hStep(1), vStep(1)
	{
	}

	SizeRange(const Size &minSize, const Size &maxSize,
		  unsigned int hstep, unsigned int vstep)
		: min(minSize), max(maxSize), hStep(hstep), vStep(vstep)
	{
	}

	bool contains(const Size &size) const;

	std::string toString() const;

	Size min;
	Size max;
	unsigned int hStep;
	unsigned int vStep;
};

bool operator==(const SizeRange &lhs, const SizeRange &rhs);
static inline bool operator!=(const SizeRange &lhs, const SizeRange &rhs)
{
	return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &out, const SizeRange &sr);

class Rectangle
{
public:
	constexpr Rectangle()
		: Rectangle(0, 0, 0, 0)
	{
	}

	constexpr Rectangle(int xpos, int ypos, const Size &size)
		: x(xpos), y(ypos), width(size.width), height(size.height)
	{
	}

	constexpr Rectangle(int xpos, int ypos, unsigned int w, unsigned int h)
		: x(xpos), y(ypos), width(w), height(h)
	{
	}

	constexpr explicit Rectangle(const Size &size)
		: x(0), y(0), width(size.width), height(size.height)
	{
	}

	int x;
	int y;
	unsigned int width;
	unsigned int height;

	bool isNull() const { return !width && !height; }
	const std::string toString() const;

	Point center() const;

	Size size() const
	{
		return { width, height };
	}

	Point topLeft() const
	{
		return { x, y };
	}

	Rectangle &scaleBy(const Size &numerator, const Size &denominator);
	Rectangle &translateBy(const Point &point);

	__nodiscard Rectangle boundedTo(const Rectangle &bound) const;
	__nodiscard Rectangle enclosedIn(const Rectangle &boundary) const;
	__nodiscard Rectangle scaledBy(const Size &numerator,
				       const Size &denominator) const;
	__nodiscard Rectangle translatedBy(const Point &point) const;
};

bool operator==(const Rectangle &lhs, const Rectangle &rhs);
static inline bool operator!=(const Rectangle &lhs, const Rectangle &rhs)
{
	return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &out, const Rectangle &r);

} /* namespace libcamera */
