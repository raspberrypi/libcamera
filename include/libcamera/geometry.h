/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.h - Geometry-related classes
 */

#ifndef __LIBCAMERA_GEOMETRY_H__
#define __LIBCAMERA_GEOMETRY_H__

#include <algorithm>
#include <string>

namespace libcamera {

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

	constexpr Size alignedDownTo(unsigned int hAlignment,
				     unsigned int vAlignment) const
	{
		return {
			width / hAlignment * hAlignment,
			height / vAlignment * vAlignment
		};
	}

	constexpr Size alignedUpTo(unsigned int hAlignment,
				   unsigned int vAlignment) const
	{
		return {
			(width + hAlignment - 1) / hAlignment * hAlignment,
			(height + vAlignment - 1) / vAlignment * vAlignment
		};
	}

	constexpr Size boundedTo(const Size &bound) const
	{
		return {
			std::min(width, bound.width),
			std::min(height, bound.height)
		};
	}

	constexpr Size expandedTo(const Size &expand) const
	{
		return {
			std::max(width, expand.width),
			std::max(height, expand.height)
		};
	}
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

	int x;
	int y;
	unsigned int width;
	unsigned int height;

	bool isNull() const { return !width && !height; }
	const std::string toString() const;
};

bool operator==(const Rectangle &lhs, const Rectangle &rhs);
static inline bool operator!=(const Rectangle &lhs, const Rectangle &rhs)
{
	return !(lhs == rhs);
}

} /* namespace libcamera */

#endif /* __LIBCAMERA_GEOMETRY_H__ */
