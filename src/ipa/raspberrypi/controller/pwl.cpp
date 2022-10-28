/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * pwl.cpp - piecewise linear functions
 */

#include <cassert>
#include <cmath>
#include <stdexcept>

#include "pwl.h"

using namespace RPiController;

int Pwl::read(const libcamera::YamlObject &params)
{
	if (!params.size() || params.size() % 2)
		return -EINVAL;

	const auto &list = params.asList();

	for (auto it = list.begin(); it != list.end(); it++) {
		auto x = it->get<double>();
		if (!x)
			return -EINVAL;
		if (it != list.begin() && *x <= points_.back().x)
			return -EINVAL;

		auto y = (++it)->get<double>();
		if (!y)
			return -EINVAL;

		points_.push_back(Point(*x, *y));
	}

	return 0;
}

void Pwl::append(double x, double y, const double eps)
{
	if (points_.empty() || points_.back().x + eps < x)
		points_.push_back(Point(x, y));
}

void Pwl::prepend(double x, double y, const double eps)
{
	if (points_.empty() || points_.front().x - eps > x)
		points_.insert(points_.begin(), Point(x, y));
}

Pwl::Interval Pwl::domain() const
{
	return Interval(points_[0].x, points_[points_.size() - 1].x);
}

Pwl::Interval Pwl::range() const
{
	double lo = points_[0].y, hi = lo;
	for (auto &p : points_)
		lo = std::min(lo, p.y), hi = std::max(hi, p.y);
	return Interval(lo, hi);
}

bool Pwl::empty() const
{
	return points_.empty();
}

double Pwl::eval(double x, int *spanPtr, bool updateSpan) const
{
	int span = findSpan(x, spanPtr && *spanPtr != -1 ? *spanPtr : points_.size() / 2 - 1);
	if (spanPtr && updateSpan)
		*spanPtr = span;
	return points_[span].y +
	       (x - points_[span].x) * (points_[span + 1].y - points_[span].y) /
		       (points_[span + 1].x - points_[span].x);
}

int Pwl::findSpan(double x, int span) const
{
	/*
	 * Pwls are generally small, so linear search may well be faster than
	 * binary, though could review this if large PWls start turning up.
	 */
	int lastSpan = points_.size() - 2;
	/*
	 * some algorithms may call us with span pointing directly at the last
	 * control point
	 */
	span = std::max(0, std::min(lastSpan, span));
	while (span < lastSpan && x >= points_[span + 1].x)
		span++;
	while (span && x < points_[span].x)
		span--;
	return span;
}

Pwl::PerpType Pwl::invert(Point const &xy, Point &perp, int &span,
			  const double eps) const
{
	assert(span >= -1);
	bool prevOffEnd = false;
	for (span = span + 1; span < (int)points_.size() - 1; span++) {
		Point spanVec = points_[span + 1] - points_[span];
		double t = ((xy - points_[span]) % spanVec) / spanVec.len2();
		if (t < -eps) /* off the start of this span */
		{
			if (span == 0) {
				perp = points_[span];
				return PerpType::Start;
			} else if (prevOffEnd) {
				perp = points_[span];
				return PerpType::Vertex;
			}
		} else if (t > 1 + eps) /* off the end of this span */
		{
			if (span == (int)points_.size() - 2) {
				perp = points_[span + 1];
				return PerpType::End;
			}
			prevOffEnd = true;
		} else /* a true perpendicular */
		{
			perp = points_[span] + spanVec * t;
			return PerpType::Perpendicular;
		}
	}
	return PerpType::None;
}

Pwl Pwl::inverse(bool *trueInverse, const double eps) const
{
	bool appended = false, prepended = false, neither = false;
	Pwl inverse;

	for (Point const &p : points_) {
		if (inverse.empty())
			inverse.append(p.y, p.x, eps);
		else if (std::abs(inverse.points_.back().x - p.y) <= eps ||
			 std::abs(inverse.points_.front().x - p.y) <= eps)
			/* do nothing */;
		else if (p.y > inverse.points_.back().x) {
			inverse.append(p.y, p.x, eps);
			appended = true;
		} else if (p.y < inverse.points_.front().x) {
			inverse.prepend(p.y, p.x, eps);
			prepended = true;
		} else
			neither = true;
	}

	/*
	 * This is not a proper inverse if we found ourselves putting points
	 * onto both ends of the inverse, or if there were points that couldn't
	 * go on either.
	 */
	if (trueInverse)
		*trueInverse = !(neither || (appended && prepended));

	return inverse;
}

Pwl Pwl::compose(Pwl const &other, const double eps) const
{
	double thisX = points_[0].x, thisY = points_[0].y;
	int thisSpan = 0, otherSpan = other.findSpan(thisY, 0);
	Pwl result({ { thisX, other.eval(thisY, &otherSpan, false) } });
	while (thisSpan != (int)points_.size() - 1) {
		double dx = points_[thisSpan + 1].x - points_[thisSpan].x,
		       dy = points_[thisSpan + 1].y - points_[thisSpan].y;
		if (std::abs(dy) > eps &&
		    otherSpan + 1 < (int)other.points_.size() &&
		    points_[thisSpan + 1].y >=
			    other.points_[otherSpan + 1].x + eps) {
			/*
			 * next control point in result will be where this
			 * function's y reaches the next span in other
			 */
			thisX = points_[thisSpan].x +
				(other.points_[otherSpan + 1].x -
				 points_[thisSpan].y) *
					dx / dy;
			thisY = other.points_[++otherSpan].x;
		} else if (std::abs(dy) > eps && otherSpan > 0 &&
			   points_[thisSpan + 1].y <=
				   other.points_[otherSpan - 1].x - eps) {
			/*
			 * next control point in result will be where this
			 * function's y reaches the previous span in other
			 */
			thisX = points_[thisSpan].x +
				(other.points_[otherSpan + 1].x -
				 points_[thisSpan].y) *
					dx / dy;
			thisY = other.points_[--otherSpan].x;
		} else {
			/* we stay in the same span in other */
			thisSpan++;
			thisX = points_[thisSpan].x,
			thisY = points_[thisSpan].y;
		}
		result.append(thisX, other.eval(thisY, &otherSpan, false),
			      eps);
	}
	return result;
}

void Pwl::map(std::function<void(double x, double y)> f) const
{
	for (auto &pt : points_)
		f(pt.x, pt.y);
}

void Pwl::map2(Pwl const &pwl0, Pwl const &pwl1,
	       std::function<void(double x, double y0, double y1)> f)
{
	int span0 = 0, span1 = 0;
	double x = std::min(pwl0.points_[0].x, pwl1.points_[0].x);
	f(x, pwl0.eval(x, &span0, false), pwl1.eval(x, &span1, false));
	while (span0 < (int)pwl0.points_.size() - 1 ||
	       span1 < (int)pwl1.points_.size() - 1) {
		if (span0 == (int)pwl0.points_.size() - 1)
			x = pwl1.points_[++span1].x;
		else if (span1 == (int)pwl1.points_.size() - 1)
			x = pwl0.points_[++span0].x;
		else if (pwl0.points_[span0 + 1].x > pwl1.points_[span1 + 1].x)
			x = pwl1.points_[++span1].x;
		else
			x = pwl0.points_[++span0].x;
		f(x, pwl0.eval(x, &span0, false), pwl1.eval(x, &span1, false));
	}
}

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

void Pwl::matchDomain(Interval const &domain, bool clip, const double eps)
{
	int span = 0;
	prepend(domain.start, eval(clip ? points_[0].x : domain.start, &span),
		eps);
	span = points_.size() - 2;
	append(domain.end, eval(clip ? points_.back().x : domain.end, &span),
	       eps);
}

Pwl &Pwl::operator*=(double d)
{
	for (auto &pt : points_)
		pt.y *= d;
	return *this;
}

void Pwl::debug(FILE *fp) const
{
	fprintf(fp, "Pwl {\n");
	for (auto &p : points_)
		fprintf(fp, "\t(%g, %g)\n", p.x, p.y);
	fprintf(fp, "}\n");
}
