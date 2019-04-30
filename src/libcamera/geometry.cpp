/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.cpp - Geometry-related structures
 */

#include <libcamera/geometry.h>

#include <sstream>
#include <stdint.h>

/**
 * \file geometry.h
 * \brief Data structures related to geometric objects
 */

namespace libcamera {

/**
 * \struct Rectangle
 * \brief Describe a rectangle's position and dimensions
 *
 * Rectangles are used to identify an area of an image. They are specified by
 * the coordinates of top-left corner and their horizontal and vertical size.
 *
 * The measure unit of the rectangle coordinates and size, as well as the
 * reference point from which the Rectangle::x and Rectangle::y displacements
 * refers to, are defined by the context were rectangle is used.
 */

/**
 * \var Rectangle::x
 * \brief The horizontal coordinate of the rectangle's top-left corner
 */

/**
 * \var Rectangle::y
 * \brief The vertical coordinate of the rectangle's top-left corner
 */

/**
 * \var Rectangle::w
 * \brief The distance between the left and right sides
 */

/**
 * \var Rectangle::h
 * \brief The distance between the top and bottom sides
 */

/**
 * \brief Assemble and return a string describing the rectangle
 * \return A string describing the Rectangle
 */
const std::string Rectangle::toString() const
{
	std::stringstream ss;

	ss << "(" << x << "x" << y << ")/" << w << "x" << h;

	return ss.str();
}

/**
 * \brief Compare rectangles for equality
 * \return True if the two rectangles are equal, false otherwise
 */
bool operator==(const Rectangle &lhs, const Rectangle &rhs)
{
	return lhs.x == rhs.x && lhs.y == rhs.y &&
	       lhs.w == rhs.w && lhs.h == rhs.h;
}

/**
 * \fn bool operator!=(const Rectangle &lhs, const Rectangle &rhs)
 * \brief Compare rectangles for inequality
 * \return True if the two rectangles are not equal, false otherwise
 */

/**
 * \struct Size
 * \brief Describe a two-dimensional size
 *
 * The Size structure defines a two-dimensional size with integer precision.
 */

/**
 * \fn Size::Size()
 * \brief Construct a Size with width and height set to 0
 */

/**
 * \fn Size::Size(unsigned int width, unsigned int height)
 * \brief Construct a Size with given \a width and \a height
 * \param[in] width The Size width
 * \param[in] height The Size height
 */

/**
 * \var Size::width
 * \brief The Size width
 */

/**
 * \var Size::height
 * \brief The Size height
 */

/**
 * \brief Assemble and return a string describing the size
 * \return A string describing the size
 */
const std::string Size::toString() const
{
	return std::to_string(width) + "x" + std::to_string(height);
}

/**
 * \brief Compare sizes for equality
 * \return True if the two sizes are equal, false otherwise
 */
bool operator==(const Size &lhs, const Size &rhs)
{
	return lhs.width == rhs.width && lhs.height == rhs.height;
}

/**
 * \brief Compare sizes for smaller than order
 *
 * Sizes are compared on three criteria, in the following order.
 *
 * - A size with smaller width and smaller height is smaller.
 * - A size with smaller area is smaller.
 * - A size with smaller width is smaller.
 *
 * \return True if \a lhs is smaller than \a rhs, false otherwise
 */
bool operator<(const Size &lhs, const Size &rhs)
{
	if (lhs.width < rhs.width && lhs.height < rhs.height)
		return true;
	else if (lhs.width >= rhs.width && lhs.height >= rhs.height)
		return false;

	uint64_t larea = static_cast<uint64_t>(lhs.width) *
			 static_cast<uint64_t>(lhs.height);
	uint64_t rarea = static_cast<uint64_t>(rhs.width) *
			 static_cast<uint64_t>(rhs.height);
	if (larea < rarea)
		return true;
	else if (larea > rarea)
		return false;

	return lhs.width < rhs.width;
}

/**
 * \fn bool operator!=(const Size &lhs, const Size &rhs)
 * \brief Compare sizes for inequality
 * \return True if the two sizes are not equal, false otherwise
 */

/**
 * \fn bool operator<=(const Size &lhs, const Size &rhs)
 * \brief Compare sizes for smaller than or equal to order
 * \return True if \a lhs is smaller than or equal to \a rhs, false otherwise
 * \sa bool operator<(const Size &lhs, const Size &rhs)
 */

/**
 * \fn bool operator>(const Size &lhs, const Size &rhs)
 * \brief Compare sizes for greater than order
 * \return True if \a lhs is greater than \a rhs, false otherwise
 * \sa bool operator<(const Size &lhs, const Size &rhs)
 */

/**
 * \fn bool operator>=(const Size &lhs, const Size &rhs)
 * \brief Compare sizes for greater than or equal to order
 * \return True if \a lhs is greater than or equal to \a rhs, false otherwise
 * \sa bool operator<(const Size &lhs, const Size &rhs)
 */

/**
 * \struct SizeRange
 * \brief Describe a range of sizes
 *
 * SizeRange describes a range of sizes included in the [min, max]
 * interval for both the width and the height. If the minimum and
 * maximum sizes are identical it represents a single size.
 */

/**
 * \fn SizeRange::SizeRange()
 * \brief Construct a size range initialized to 0
 */

/**
 * \fn SizeRange::SizeRange(unsigned int minW, unsigned int minH, unsigned int maxW, unsigned int maxH)
 * \brief Construct an initialized size range
 * \param[in] minW The minimum width
 * \param[in] minH The minimum height
 * \param[in] maxW The maximum width
 * \param[in] maxH The maximum height
 */

/**
 * \var SizeRange::min
 * \brief The minimum size
 */

/**
 * \var SizeRange::max
 * \brief The maximum size
 */

/**
 * \brief Compare size ranges for equality
 * \return True if the two size ranges are equal, false otherwise
 */
bool operator==(const SizeRange &lhs, const SizeRange &rhs)
{
	return lhs.min == rhs.min && lhs.max == rhs.max;
}

/**
 * \fn bool operator!=(const SizeRange &lhs, const SizeRange &rhs)
 * \brief Compare size ranges for inequality
 * \return True if the two size ranges are not equal, false otherwise
 */

} /* namespace libcamera */
