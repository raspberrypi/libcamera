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
 * \fn bool Size::isNull() const
 * \brief Check if the size is null
 * \return True if both the width and height are 0, or false otherwise
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
 * \fn Size::alignDownTo(unsigned int hAlignment, unsigned int vAlignment)
 * \brief Align the size down horizontally and vertically in place
 * \param[in] hAlignment Horizontal alignment
 * \param[in] vAlignment Vertical alignment
 *
 * This functions rounds the width and height down to the nearest multiple of
 * \a hAlignment and \a vAlignment respectively.
 *
 * \return A reference to this object
 */

/**
 * \fn Size::alignUpTo(unsigned int hAlignment, unsigned int vAlignment)
 * \brief Align the size up horizontally and vertically in place
 * \param[in] hAlignment Horizontal alignment
 * \param[in] vAlignment Vertical alignment
 *
 * This functions rounds the width and height up to the nearest multiple of
 * \a hAlignment and \a vAlignment respectively.
 *
 * \return A reference to this object
 */

/**
 * \fn Size::boundTo(const Size &bound)
 * \brief Bound the size to \a bound in place
 * \param[in] bound The maximum size
 *
 * This function sets the width and height to the minimum of this size and the
 * \a bound size.
 *
 * \return A reference to this object
 */

/**
 * \fn Size::expandTo(const Size &expand)
 * \brief Expand the size to \a expand
 * \param[in] expand The minimum size
 *
 * This function sets the width and height to the maximum of this size and the
 * \a expand size.
 *
 * \return A reference to this object
 */

/**
 * \fn Size::alignedDownTo(unsigned int hAlignment, unsigned int vAlignment)
 * \brief Align the size down horizontally and vertically
 * \param[in] hAlignment Horizontal alignment
 * \param[in] vAlignment Vertical alignment
 * \return A Size whose width and height are equal to the width and height of
 * this size rounded down to the nearest multiple of \a hAlignment and
 * \a vAlignment respectively
 */

/**
 * \fn Size::alignedUpTo(unsigned int hAlignment, unsigned int vAlignment)
 * \brief Align the size up horizontally and vertically
 * \param[in] hAlignment Horizontal alignment
 * \param[in] vAlignment Vertical alignment
 * \return A Size whose width and height are equal to the width and height of
 * this size rounded up to the nearest multiple of \a hAlignment and
 * \a vAlignment respectively
 */

/**
 * \fn Size::boundedTo(const Size &bound)
 * \brief Bound the size to \a bound
 * \param[in] bound The maximum size
 * \return A Size whose width and height are the minimum of the width and
 * height of this size and the \a bound size
 */

/**
 * \fn Size::expandedTo(const Size &expand)
 * \brief Expand the size to \a expand
 * \param[in] expand The minimum size
 * \return A Size whose width and height are the maximum of the width and
 * height of this size and the \a expand size
 */

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
 * A SizeRange describes a range of sizes included in the [min, max] interval
 * for both the width and the height. If the minimum and maximum sizes are
 * identical it represents a single size.
 *
 * Size ranges may further limit the valid sizes through steps in the horizontal
 * and vertical direction. The step values represent the increase in pixels
 * between two valid width or height values, starting from the minimum. Valid
 * sizes within the range are thus expressed as
 *
 *	width = min.width + hStep * x
 *	height = min.height + vStep * y
 *
 *	Where
 *
 *	width <= max.width
 *	height < max.height
 *
 * Note that the step values are not equivalent to alignments, as the minimum
 * width or height may not be a multiple of the corresponding step.
 *
 * The step values may be zero when the range describes only minimum and
 * maximum sizes without implying that all, or any, intermediate size is valid.
 * SizeRange instances the describe a single size have both set values set to 1.
 */

/**
 * \fn SizeRange::SizeRange()
 * \brief Construct a size range initialized to 0
 */

/**
 * \fn SizeRange::SizeRange(const Size &size)
 * \brief Construct a size range representing a single size
 * \param[in] size The size
 */

/**
 * \fn SizeRange::SizeRange(const Size &minSize, const Size &maxSize)
 * \brief Construct a size range with specified min and max, and steps of 1
 * \param[in] minSize The minimum size
 * \param[in] maxSize The maximum size
 */

/**
 * \fn SizeRange::SizeRange(const Size &minSize, const Size &maxSize,
 *			    unsigned int hstep, unsigned int vstep)
 * \brief Construct a size range with specified min, max and step
 * \param[in] minSize The minimum size
 * \param[in] maxSize The maximum size
 * \param[in] hstep The horizontal step
 * \param[in] vstep The vertical step
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
 * \var SizeRange::hStep
 * \brief The horizontal step
 */

/**
 * \var SizeRange::vStep
 * \brief The vertical step
 */

/**
 * \brief Test if a size is contained in the range
 * \param[in] size Size to check
 * \return True if \a size is contained in the range
 */
bool SizeRange::contains(const Size &size) const
{
	if (size.width < min.width || size.width > max.width ||
	    size.height < min.height || size.height > max.height ||
	    (hStep && (size.width - min.width) % hStep) ||
	    (vStep && (size.height - min.height) % vStep))
		return false;

	return true;
}

/**
 * \brief Assemble and return a string describing the size range
 * \return A string describing the SizeRange
 */
std::string SizeRange::toString() const
{
	std::stringstream ss;

	ss << "(" << min.toString() << ")-(" << max.toString() << ")/(+"
	   << hStep << ",+" << vStep << ")";

	return ss.str();
}

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
 * \fn Rectangle::Rectangle()
 * \brief Construct a Rectangle with all coordinates set to 0
 */

/**
 * \fn Rectangle::Rectangle(int x, int y, const Size &size)
 * \brief Construct a Rectangle with the given position and size
 * \param[in] x The horizontal coordinate of the top-left corner
 * \param[in] y The vertical coordinate of the top-left corner
 * \param[in] size The size
 */

/**
 * \fn Rectangle::Rectangle(int x, int y, unsigned int width, unsigned int height)
 * \brief Construct a Rectangle with the given position and size
 * \param[in] x The horizontal coordinate of the top-left corner
 * \param[in] y The vertical coordinate of the top-left corner
 * \param[in] width The width
 * \param[in] height The height
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
 * \var Rectangle::width
 * \brief The distance between the left and right sides
 */

/**
 * \var Rectangle::height
 * \brief The distance between the top and bottom sides
 */

/**
 * \fn bool Rectangle::isNull() const
 * \brief Check if the rectangle is null
 * \return True if both the width and height are 0, or false otherwise
 */

/**
 * \brief Assemble and return a string describing the rectangle
 * \return A string describing the Rectangle
 */
const std::string Rectangle::toString() const
{
	std::stringstream ss;

	ss << "(" << x << "x" << y << ")/" << width << "x" << height;

	return ss.str();
}

/**
 * \brief Compare rectangles for equality
 * \return True if the two rectangles are equal, false otherwise
 */
bool operator==(const Rectangle &lhs, const Rectangle &rhs)
{
	return lhs.x == rhs.x && lhs.y == rhs.y &&
	       lhs.width == rhs.width && lhs.height == rhs.height;
}

/**
 * \fn bool operator!=(const Rectangle &lhs, const Rectangle &rhs)
 * \brief Compare rectangles for inequality
 * \return True if the two rectangles are not equal, false otherwise
 */

} /* namespace libcamera */
