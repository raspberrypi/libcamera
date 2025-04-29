/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * Piecewise linear functions interface
 */
#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "libcamera/internal/vector.h"

namespace libcamera {

namespace ipa {

class Pwl
{
public:
	using Point = Vector<double, 2>;

	struct Interval {
		Interval(double _start, double _end)
			: start(_start), end(_end) {}

		bool contains(double value)
		{
			return value >= start && value <= end;
		}

		double clamp(double value)
		{
			return std::clamp(value, start, end);
		}

		double length() const { return end - start; }

		double start, end;
	};

	Pwl();
	Pwl(const std::vector<Point> &points);
	Pwl(std::vector<Point> &&points);

	void append(double x, double y, double eps = 1e-6);

	bool empty() const { return points_.empty(); }
	void clear() { points_.clear(); }
	size_t size() const { return points_.size(); }

	Interval domain() const;
	Interval range() const;

	double eval(double x, int *span = nullptr,
		    bool updateSpan = true) const;

	std::pair<Pwl, bool> inverse(double eps = 1e-6) const;
	Pwl compose(const Pwl &other, double eps = 1e-6) const;

	void map(std::function<void(double x, double y)> f) const;

	static Pwl
	combine(const Pwl &pwl0, const Pwl &pwl1,
		std::function<double(double x, double y0, double y1)> f,
		double eps = 1e-6);

	Pwl &operator*=(double d);

	std::string toString() const;

private:
	static void map2(const Pwl &pwl0, const Pwl &pwl1,
			 std::function<void(double x, double y0, double y1)> f);
	void prepend(double x, double y, double eps = 1e-6);
	int findSpan(double x, int span) const;

	std::vector<Point> points_;
};

} /* namespace ipa */

} /* namespace libcamera */
