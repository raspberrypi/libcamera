/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Piecewise linear functions
 */

#include "pwl.h"

#include <cmath>
#include <sstream>

/**
 * \file pwl.h
 * \brief Piecewise linear functions
 */

namespace libcamera {

namespace ipa {

/**
 * \class Pwl
 * \brief Describe a univariate piecewise linear function in two-dimensional
 * real space
 *
 * A piecewise linear function is a univariate function that maps reals to
 * reals, and it is composed of multiple straight-line segments.
 *
 * While a mathematical piecewise linear function would usually be defined by
 * a list of linear functions and for which values of the domain they apply,
 * this Pwl class is instead defined by a list of points at which these line
 * segments intersect. These intersecting points are known as knots.
 *
 * https://en.wikipedia.org/wiki/Piecewise_linear_function
 *
 * A consequence of the Pwl class being defined by knots instead of linear
 * functions is that the values of the piecewise linear function past the ends
 * of the function are constants as opposed to linear functions. In a
 * mathematical piecewise linear function that is defined by multiple linear
 * functions, the ends of the function are also linear functions and hence grow
 * to infinity (or negative infinity). However, since this Pwl class is defined
 * by knots, the y-value of the leftmost and rightmost knots will hold for all
 * x values to negative infinity and positive infinity, respectively.
 */

/**
 * \typedef Pwl::Point
 * \brief Describe a point in two-dimensional real space
 */

/**
 * \class Pwl::Interval
 * \brief Describe an interval in one-dimensional real space
 */

/**
 * \fn Pwl::Interval::Interval(double _start, double _end)
 * \brief Construct an interval
 * \param[in] _start Start of the interval
 * \param[in] _end End of the interval
 */

/**
 * \fn Pwl::Interval::contains
 * \brief Check if a given value falls within the interval
 * \param[in] value Value to check
 * \return True if the value falls within the interval, including its bounds,
 * or false otherwise
 */

/**
 * \fn Pwl::Interval::clamp
 * \brief Clamp a value such that it is within the interval
 * \param[in] value Value to clamp
 * \return The clamped value
 */

/**
 * \fn Pwl::Interval::length
 * \brief Compute the length of the interval
 * \return The length of the interval
 */

/**
 * \var Pwl::Interval::start
 * \brief Start of the interval
 */

/**
 * \var Pwl::Interval::end
 * \brief End of the interval
 */

/**
 * \brief Construct an empty piecewise linear function
 */
Pwl::Pwl()
{
}

/**
 * \brief Construct a piecewise linear function from a list of 2D points
 * \param[in] points Vector of points from which to construct the piecewise
 * linear function
 *
 * \a points must be in ascending order of x-value.
 */
Pwl::Pwl(const std::vector<Point> &points)
	: points_(points)
{
}

/**
 * \copydoc Pwl::Pwl(const std::vector<Point> &points)
 *
 * The contents of the \a points vector is moved to the newly constructed Pwl
 * instance.
 */
Pwl::Pwl(std::vector<Point> &&points)
	: points_(std::move(points))
{
}

/**
 * \brief Append a point to the end of the piecewise linear function
 * \param[in] x x-coordinate of the point to add to the piecewise linear function
 * \param[in] y y-coordinate of the point to add to the piecewise linear function
 * \param[in] eps Epsilon for the minimum x distance between points (optional)
 *
 * The point's x-coordinate must be greater than the x-coordinate of the last
 * (= greatest) point already in the piecewise linear function.
 */
void Pwl::append(double x, double y, const double eps)
{
	if (points_.empty() || points_.back().x() + eps < x)
		points_.push_back(Point({ x, y }));
}

/**
 * \brief Prepend a point to the beginning of the piecewise linear function
 * \param[in] x x-coordinate of the point to add to the piecewise linear function
 * \param[in] y y-coordinate of the point to add to the piecewise linear function
 * \param[in] eps Epsilon for the minimum x distance between points (optional)
 *
 * The point's x-coordinate must be less than the x-coordinate of the first
 * (= smallest) point already in the piecewise linear function.
 */
void Pwl::prepend(double x, double y, const double eps)
{
	if (points_.empty() || points_.front().x() - eps > x)
		points_.insert(points_.begin(), Point({ x, y }));
}

/**
 * \fn Pwl::empty() const
 * \brief Check if the piecewise linear function is empty
 * \return True if there are no points in the function, false otherwise
 */

/**
 * \fn Pwl::clear()
 * \brief Clear the piecewise linear function
 */

/**
 * \fn Pwl::size() const
 * \brief Retrieve the number of points in the piecewise linear function
 * \return The number of points in the piecewise linear function
 */

/**
 * \brief Get the domain of the piecewise linear function
 * \return An interval representing the domain
 */
Pwl::Interval Pwl::domain() const
{
	return Interval(points_[0].x(), points_[points_.size() - 1].x());
}

/**
 * \brief Get the range of the piecewise linear function
 * \return An interval representing the range
 */
Pwl::Interval Pwl::range() const
{
	double lo = points_[0].y(), hi = lo;
	for (auto &p : points_)
		lo = std::min(lo, p.y()), hi = std::max(hi, p.y());
	return Interval(lo, hi);
}

/**
 * \brief Evaluate the piecewise linear function
 * \param[in] x The x value to input into the function
 * \param[inout] span Initial guess for span
 * \param[in] updateSpan Set to true to update span
 *
 * Evaluate Pwl, optionally supplying an initial guess for the
 * "span". The "span" may be optionally be updated. If you want to know
 * the "span" value but don't have an initial guess you can set it to
 * -1.
 *
 *  \return The result of evaluating the piecewise linear function at position \a x
 */
double Pwl::eval(double x, int *span, bool updateSpan) const
{
	int index = findSpan(x, span && *span != -1
					? *span
					: points_.size() / 2 - 1);
	if (span && updateSpan)
		*span = index;
	return points_[index].y() +
	       (x - points_[index].x()) * (points_[index + 1].y() - points_[index].y()) /
		       (points_[index + 1].x() - points_[index].x());
}

int Pwl::findSpan(double x, int span) const
{
	/*
	 * Pwls are generally small, so linear search may well be faster than
	 * binary, though could review this if large Pwls start turning up.
	 */
	int lastSpan = points_.size() - 2;
	/*
	 * some algorithms may call us with span pointing directly at the last
	 * control point
	 */
	span = std::max(0, std::min(lastSpan, span));
	while (span < lastSpan && x >= points_[span + 1].x())
		span++;
	while (span && x < points_[span].x())
		span--;
	return span;
}

/**
 * \brief Compute the inverse function
 * \param[in] eps Epsilon for the minimum x distance between points (optional)
 *
 * The output includes whether the resulting inverse function is a proper
 * (true) inverse, or only a best effort (e.g. input was non-monotonic).
 *
 * \return A pair of the inverse piecewise linear function, and whether or not
 * the result is a proper/true inverse
 */
std::pair<Pwl, bool> Pwl::inverse(const double eps) const
{
	bool appended = false, prepended = false, neither = false;
	Pwl inverse;

	for (Point const &p : points_) {
		if (inverse.empty()) {
			inverse.append(p.y(), p.x(), eps);
		} else if (std::abs(inverse.points_.back().x() - p.y()) <= eps ||
			   std::abs(inverse.points_.front().x() - p.y()) <= eps) {
			/* do nothing */;
		} else if (p.y() > inverse.points_.back().x()) {
			inverse.append(p.y(), p.x(), eps);
			appended = true;
		} else if (p.y() < inverse.points_.front().x()) {
			inverse.prepend(p.y(), p.x(), eps);
			prepended = true;
		} else {
			neither = true;
		}
	}

	/*
	 * This is not a proper inverse if we found ourselves putting points
	 * onto both ends of the inverse, or if there were points that couldn't
	 * go on either.
	 */
	bool trueInverse = !(neither || (appended && prepended));

	return { inverse, trueInverse };
}

/**
 * \brief Compose two piecewise linear functions together
 * \param[in] other The "other" piecewise linear function
 * \param[in] eps Epsilon for the minimum x distance between points (optional)
 *
 * The "this" function is done first, and "other" after.
 *
 * \return The composed piecewise linear function
 */
Pwl Pwl::compose(Pwl const &other, const double eps) const
{
	double thisX = points_[0].x(), thisY = points_[0].y();
	int thisSpan = 0, otherSpan = other.findSpan(thisY, 0);
	Pwl result({ Point({ thisX, other.eval(thisY, &otherSpan, false) }) });

	while (thisSpan != (int)points_.size() - 1) {
		double dx = points_[thisSpan + 1].x() - points_[thisSpan].x(),
		       dy = points_[thisSpan + 1].y() - points_[thisSpan].y();
		if (std::abs(dy) > eps &&
		    otherSpan + 1 < (int)other.points_.size() &&
		    points_[thisSpan + 1].y() >= other.points_[otherSpan + 1].x() + eps) {
			/*
			 * next control point in result will be where this
			 * function's y reaches the next span in other
			 */
			thisX = points_[thisSpan].x() +
				(other.points_[otherSpan + 1].x() -
				 points_[thisSpan].y()) *
					dx / dy;
			thisY = other.points_[++otherSpan].x();
		} else if (std::abs(dy) > eps && otherSpan > 0 &&
			   points_[thisSpan + 1].y() <=
				   other.points_[otherSpan - 1].x() - eps) {
			/*
			 * next control point in result will be where this
			 * function's y reaches the previous span in other
			 */
			thisX = points_[thisSpan].x() +
				(other.points_[otherSpan + 1].x() -
				 points_[thisSpan].y()) *
					dx / dy;
			thisY = other.points_[--otherSpan].x();
		} else {
			/* we stay in the same span in other */
			thisSpan++;
			thisX = points_[thisSpan].x(),
			thisY = points_[thisSpan].y();
		}
		result.append(thisX, other.eval(thisY, &otherSpan, false),
			      eps);
	}
	return result;
}

/**
 * \brief Apply function to (x, y) values at every control point
 * \param[in] f Function to be applied
 */
void Pwl::map(std::function<void(double x, double y)> f) const
{
	for (auto &pt : points_)
		f(pt.x(), pt.y());
}

/**
 * \brief Apply function to (x, y0, y1) values wherever either Pwl has a
 * control point.
 * \param[in] pwl0 First piecewise linear function
 * \param[in] pwl1 Second piecewise linear function
 * \param[in] f Function to be applied
 *
 * This applies the function \a f to every parameter (x, y0, y1), where x is
 * the combined list of x-values from \a pwl0 and \a pwl1, y0 is the y-value
 * for the given x in \a pwl0, and y1 is the y-value for the same x in \a pwl1.
 */
void Pwl::map2(Pwl const &pwl0, Pwl const &pwl1,
	       std::function<void(double x, double y0, double y1)> f)
{
	int span0 = 0, span1 = 0;
	double x = std::min(pwl0.points_[0].x(), pwl1.points_[0].x());
	f(x, pwl0.eval(x, &span0, false), pwl1.eval(x, &span1, false));

	while (span0 < (int)pwl0.points_.size() - 1 ||
	       span1 < (int)pwl1.points_.size() - 1) {
		if (span0 == (int)pwl0.points_.size() - 1)
			x = pwl1.points_[++span1].x();
		else if (span1 == (int)pwl1.points_.size() - 1)
			x = pwl0.points_[++span0].x();
		else if (pwl0.points_[span0 + 1].x() > pwl1.points_[span1 + 1].x())
			x = pwl1.points_[++span1].x();
		else
			x = pwl0.points_[++span0].x();
		f(x, pwl0.eval(x, &span0, false), pwl1.eval(x, &span1, false));
	}
}

/**
 * \brief Combine two Pwls
 * \param[in] pwl0 First piecewise linear function
 * \param[in] pwl1 Second piecewise linear function
 * \param[in] f Function to be applied
 * \param[in] eps Epsilon for the minimum x distance between points (optional)
 *
 * Create a new Pwl where the y values are given by running \a f wherever
 * either pwl has a knot.
 *
 * \return The combined pwl
 */
Pwl Pwl::combine(Pwl const &pwl0, Pwl const &pwl1,
		 std::function<double(double x, double y0, double y1)> f,
		 const double eps)
{
	Pwl result;
	map2(pwl0, pwl1, [&](double x, double y0, double y1) {
		result.append(x, f(x, y0, y1), eps);
	});
	return result;
}

/**
 * \brief Multiply the piecewise linear function
 * \param[in] d Scalar multiplier to multiply the function by
 * \return This function, after it has been multiplied by \a d
 */
Pwl &Pwl::operator*=(double d)
{
	for (auto &pt : points_)
		pt[1] *= d;
	return *this;
}

/**
 * \brief Assemble and return a string describing the piecewise linear function
 * \return A string describing the piecewise linear function
 */
std::string Pwl::toString() const
{
	std::stringstream ss;
	ss << "Pwl { ";
	for (auto &p : points_)
		ss << "(" << p.x() << ", " << p.y() << ") ";
	ss << "}";
	return ss.str();
}

} /* namespace ipa */

#ifndef __DOXYGEN__
/*
 * The YAML data shall be a list of numerical values with an even number of
 * elements. They are parsed in pairs into x and y points in the piecewise
 * linear function, and added in order. x must be monotonically increasing.
 */
template<>
std::optional<ipa::Pwl>
YamlObject::Getter<ipa::Pwl>::get(const YamlObject &obj) const
{
	if (!obj.size() || obj.size() % 2)
		return std::nullopt;

	ipa::Pwl pwl;

	const auto &list = obj.asList();

	for (auto it = list.begin(); it != list.end(); it++) {
		auto x = it->get<double>();
		if (!x)
			return std::nullopt;
		auto y = (++it)->get<double>();
		if (!y)
			return std::nullopt;

		pwl.append(*x, *y);
	}

	if (pwl.size() != obj.size() / 2)
		return std::nullopt;

	return pwl;
}
#endif /* __DOXYGEN__ */

} /* namespace libcamera */
