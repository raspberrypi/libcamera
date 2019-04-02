/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * geometry.cpp - Geometry-related structures
 */

#include <sstream>

#include "geometry.h"

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
 *
 * \return A string describing the Rectangle
 */
const std::string Rectangle::toString() const
{
	std::stringstream ss;

	ss << "(" << x << "x" << y << ")/" << w << "x" << h;

	return ss.str();
}

/**
 * \struct SizeRange
 * \brief Describe a range of image sizes
 *
 * SizeRange describes a range of image sizes included in the (minWidth,
 * minHeight) - (maxWidth, maxHeight) interval. If the minimum and
 * maximum sizes are identical it represents a single image resolution.
 */

/**
 * \fn SizeRange::SizeRange()
 * \brief Construct a size range initialized to 0
 */

/**
 * \fn SizeRange::SizeRange(unsigned int minW, unsigned int minH, unsigned int maxW, unsigned int maxH)
 * \brief Construct an initialized size range
 * \param minW The minimum width
 * \param minH The minimum height
 * \param maxW The maximum width
 * \param maxH The maximum height
 */

/**
 * \var SizeRange::minWidth
 * \brief The minimum image width
 */

/**
 * \var SizeRange::minHeight
 * \brief The minimum image height
 */

/**
 * \var SizeRange::maxWidth
 * \brief The maximum image width
 */

/**
 * \var SizeRange::maxHeight
 * \brief The maximum image height
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
 * \param width The Size width
 * \param height The Size height
 */

/**
 * \var Size::width
 * \brief The Size width
 */

/**
 * \var Size::height
 * \brief The Size height
 */

} /* namespace libcamera */
