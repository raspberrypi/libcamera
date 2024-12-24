/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Geometry classes tests
 */

#include <iostream>

#include <libcamera/geometry.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class GeometryTest : public Test
{
protected:
	template<typename T>
	bool compare(const T &lhs, const T &rhs,
		     bool (*op)(const T &lhs, const T &rhs),
		     const char *opName, bool expect)
	{
		bool result = op(lhs, rhs);

		if (result != expect) {
			cout << lhs << opName << " " << rhs
			     << "test failed" << std::endl;
			return false;
		}

		return true;
	}

	int run()
	{
		/*
		 * Point tests
		 */

		/* Equality */
		if (!compare(Point(50, 100), Point(50, 100), &operator==, "==", true))
			return TestFail;

		if (!compare(Point(-50, 100), Point(-50, 100), &operator==, "==", true))
			return TestFail;

		if (!compare(Point(50, -100), Point(50, -100), &operator==, "==", true))
			return TestFail;

		if (!compare(Point(-50, -100), Point(-50, -100), &operator==, "==", true))
			return TestFail;

		/* Inequality */
		if (!compare(Point(50, 100), Point(50, 100), &operator!=, "!=", false))
			return TestFail;

		if (!compare(Point(-50, 100), Point(-50, 100), &operator!=, "!=", false))
			return TestFail;

		if (!compare(Point(50, -100), Point(50, -100), &operator!=, "!=", false))
			return TestFail;

		if (!compare(Point(-50, -100), Point(-50, -100), &operator!=, "!=", false))
			return TestFail;

		if (!compare(Point(-50, 100), Point(50, 100), &operator!=, "!=", true))
			return TestFail;

		if (!compare(Point(50, -100), Point(50, 100), &operator!=, "!=", true))
			return TestFail;

		if (!compare(Point(-50, -100), Point(50, 100), &operator!=, "!=", true))
			return TestFail;

		/* Negation */
		if (Point(50, 100) != -Point(-50, -100) ||
		    Point(50, 100) == -Point(50, -100) ||
		    Point(50, 100) == -Point(-50, 100)) {
			cout << "Point negation test failed" << endl;
			return TestFail;
		}

		/* Default constructor */
		if (Point() != Point(0, 0)) {
			cout << "Default constructor test failed" << endl;
			return TestFail;
		}

		/*
		 * Size tests
		 */

		if (!Size().isNull() || !Size(0, 0).isNull()) {
			cout << "Null size incorrectly reported as not null" << endl;
			return TestFail;
		}

		if (Size(0, 100).isNull() || Size(100, 0).isNull() || Size(100, 100).isNull()) {
			cout << "Non-null size incorrectly reported as null" << endl;
			return TestFail;
		}

		/*
		 * Test alignDownTo(), alignUpTo(), boundTo(), expandTo(),
		 * growBy() and shrinkBy()
		 */
		Size s(50, 50);

		s.alignDownTo(16, 16);
		if (s != Size(48, 48)) {
			cout << "Size::alignDownTo() test failed" << endl;
			return TestFail;
		}

		s.alignUpTo(32, 32);
		if (s != Size(64, 64)) {
			cout << "Size::alignUpTo() test failed" << endl;
			return TestFail;
		}

		s.boundTo({ 40, 40 });
		if (s != Size(40, 40)) {
			cout << "Size::boundTo() test failed" << endl;
			return TestFail;
		}

		s.expandTo({ 50, 50 });
		if (s != Size(50, 50)) {
			cout << "Size::expandTo() test failed" << endl;
			return TestFail;
		}

		s.growBy({ 10, 20 });
		if (s != Size(60, 70)) {
			cout << "Size::growBy() test failed" << endl;
			return TestFail;
		}

		s.shrinkBy({ 20, 10 });
		if (s != Size(40, 60)) {
			cout << "Size::shrinkBy() test failed" << endl;
			return TestFail;
		}

		s.shrinkBy({ 100, 100 });
		if (s != Size(0, 0)) {
			cout << "Size::shrinkBy() clamp test failed" << endl;
			return TestFail;
		}

		s = Size(50,50).alignDownTo(16, 16).alignUpTo(32, 32)
		  .boundTo({ 40, 80 }).expandTo({ 16, 80 })
		  .growBy({ 4, 4 }).shrinkBy({ 10, 20 });
		if (s != Size(34, 64)) {
			cout << "Size chained in-place modifiers test failed" << endl;
			return TestFail;
		}

		/*
		 * Test alignedDownTo(), alignedUpTo(), boundedTo(),
		 * expandedTo(), grownBy() and shrunkBy()
		 */
		if (Size(0, 0).alignedDownTo(16, 8) != Size(0, 0) ||
		    Size(1, 1).alignedDownTo(16, 8) != Size(0, 0) ||
		    Size(16, 8).alignedDownTo(16, 8) != Size(16, 8)) {
			cout << "Size::alignedDownTo() test failed" << endl;
			return TestFail;
		}

		if (Size(0, 0).alignedUpTo(16, 8) != Size(0, 0) ||
		    Size(1, 1).alignedUpTo(16, 8) != Size(16, 8) ||
		    Size(16, 8).alignedUpTo(16, 8) != Size(16, 8)) {
			cout << "Size::alignedUpTo() test failed" << endl;
			return TestFail;
		}

		if (Size(0, 0).boundedTo({ 100, 100 }) != Size(0, 0) ||
		    Size(200, 50).boundedTo({ 100, 100 }) != Size(100, 50) ||
		    Size(50, 200).boundedTo({ 100, 100 }) != Size(50, 100)) {
			cout << "Size::boundedTo() test failed" << endl;
			return TestFail;
		}

		if (Size(0, 0).expandedTo({ 100, 100 }) != Size(100, 100) ||
		    Size(200, 50).expandedTo({ 100, 100 }) != Size(200, 100) ||
		    Size(50, 200).expandedTo({ 100, 100 }) != Size(100, 200)) {
			cout << "Size::expandedTo() test failed" << endl;
			return TestFail;
		}

		if (Size(0, 0).grownBy({ 10, 20 }) != Size(10, 20) ||
		    Size(200, 50).grownBy({ 10, 20 }) != Size(210, 70)) {
			cout << "Size::grownBy() test failed" << endl;
			return TestFail;
		}

		if (Size(200, 50).shrunkBy({ 10, 20 }) != Size(190, 30) ||
		    Size(200, 50).shrunkBy({ 10, 100 }) != Size(190, 0) ||
		    Size(200, 50).shrunkBy({ 300, 20 }) != Size(0, 30)) {
			cout << "Size::shrunkBy() test failed" << endl;
			return TestFail;
		}

		/* Aspect ratio tests */
		if (Size(0, 0).boundedToAspectRatio(Size(4, 3)) != Size(0, 0) ||
		    Size(1920, 1440).boundedToAspectRatio(Size(16, 9)) != Size(1920, 1080) ||
		    Size(1920, 1440).boundedToAspectRatio(Size(65536, 36864)) != Size(1920, 1080) ||
		    Size(1440, 1920).boundedToAspectRatio(Size(9, 16)) != Size(1080, 1920) ||
		    Size(1920, 1080).boundedToAspectRatio(Size(4, 3)) != Size(1440, 1080) ||
		    Size(1920, 1080).boundedToAspectRatio(Size(65536, 49152)) != Size(1440, 1080) ||
		    Size(1024, 1024).boundedToAspectRatio(Size(1, 1)) != Size(1024, 1024) ||
		    Size(1920, 1080).boundedToAspectRatio(Size(16, 9)) != Size(1920, 1080) ||
		    Size(200, 100).boundedToAspectRatio(Size(16, 9)) != Size(177, 100) ||
		    Size(300, 200).boundedToAspectRatio(Size(16, 9)) != Size(300, 168)) {
			cout << "Size::boundedToAspectRatio() test failed" << endl;
			return TestFail;
		}

		if (Size(0, 0).expandedToAspectRatio(Size(4, 3)) != Size(0, 0) ||
		    Size(1920, 1440).expandedToAspectRatio(Size(16, 9)) != Size(2560, 1440) ||
		    Size(1920, 1440).expandedToAspectRatio(Size(65536, 36864)) != Size(2560, 1440) ||
		    Size(1440, 1920).expandedToAspectRatio(Size(9, 16)) != Size(1440, 2560) ||
		    Size(1920, 1080).expandedToAspectRatio(Size(4, 3)) != Size(1920, 1440) ||
		    Size(1920, 1080).expandedToAspectRatio(Size(65536, 49152)) != Size(1920, 1440) ||
		    Size(1024, 1024).expandedToAspectRatio(Size(1, 1)) != Size(1024, 1024) ||
		    Size(1920, 1080).expandedToAspectRatio(Size(16, 9)) != Size(1920, 1080) ||
		    Size(200, 100).expandedToAspectRatio(Size(16, 9)) != Size(200, 112) ||
		    Size(300, 200).expandedToAspectRatio(Size(16, 9)) != Size(355, 200)) {
			cout << "Size::expandedToAspectRatio() test failed" << endl;
			return TestFail;
		}

		/* Size::centeredTo() tests */
		if (Size(0, 0).centeredTo(Point(50, 100)) != Rectangle(50, 100, 0, 0) ||
		    Size(0, 0).centeredTo(Point(-50, -100)) != Rectangle(-50, -100, 0, 0) ||
		    Size(100, 200).centeredTo(Point(50, 100)) != Rectangle(0, 0, 100, 200) ||
		    Size(100, 200).centeredTo(Point(-50, -100)) != Rectangle(-100, -200, 100, 200) ||
		    Size(101, 201).centeredTo(Point(-50, -100)) != Rectangle(-100, -200, 101, 201) ||
		    Size(101, 201).centeredTo(Point(-51, -101)) != Rectangle(-101, -201, 101, 201)) {
			cout << "Size::centeredTo() test failed" << endl;
			return TestFail;
		}

		/* Scale a size by a float */
		if (Size(1000, 2000) * 2.0 != Size(2000, 4000) ||
		    Size(300, 100) * 0.5 != Size(150, 50) ||
		    Size(1, 2) * 1.6 != Size(1, 3)) {
			cout << "Size::operator*() failed" << endl;
			return TestFail;
		}

		if (Size(1000, 2000) / 2.0 != Size(500, 1000) ||
		    Size(300, 100) / 0.5 != Size(600, 200) ||
		    Size(1000, 2000) / 3.0 != Size(333, 666)) {
			cout << "Size::operator*() failed" << endl;
			return TestFail;
		}

		s = Size(300, 100);
		s *= 0.3333;
		if (s != Size(99, 33)) {
			cout << "Size::operator*() test failed" << endl;
			return TestFail;
		}

		s = Size(300, 100);
		s /= 3;
		if (s != Size(100, 33)) {
			cout << "Size::operator*() test failed" << endl;
			return TestFail;
		}

		/* Test Size equality and inequality. */
		if (!compare(Size(100, 100), Size(100, 100), &operator==, "==", true))
			return TestFail;
		if (!compare(Size(100, 100), Size(100, 100), &operator!=, "!=", false))
			return TestFail;

		if (!compare(Size(100, 100), Size(200, 100), &operator==, "==", false))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 100), &operator!=, "!=", true))
			return TestFail;

		if (!compare(Size(100, 100), Size(100, 200), &operator==, "==", false))
			return TestFail;
		if (!compare(Size(100, 100), Size(100, 200), &operator!=, "!=", true))
			return TestFail;

		/* Test Size ordering based on combined with and height. */
		if (!compare(Size(100, 100), Size(200, 200), &operator<, "<", true))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 200), &operator<=, "<=", true))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 200), &operator>, ">", false))
			return TestFail;
		if (!compare(Size(100, 100), Size(200, 200), &operator>=, ">=", false))
			return TestFail;

		if (!compare(Size(200, 200), Size(100, 100), &operator<, "<", false))
			return TestFail;
		if (!compare(Size(200, 200), Size(100, 100), &operator<=, "<=", false))
			return TestFail;
		if (!compare(Size(200, 200), Size(100, 100), &operator>, ">", true))
			return TestFail;
		if (!compare(Size(200, 200), Size(100, 100), &operator>=, ">=", true))
			return TestFail;

		/* Test Size ordering based on area (with overlapping sizes). */
		if (!compare(Size(200, 100), Size(100, 400), &operator<, "<", true))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 400), &operator<=, "<=", true))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 400), &operator>, ">", false))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 400), &operator>=, ">=", false))
			return TestFail;

		if (!compare(Size(100, 400), Size(200, 100), &operator<, "<", false))
			return TestFail;
		if (!compare(Size(100, 400), Size(200, 100), &operator<=, "<=", false))
			return TestFail;
		if (!compare(Size(100, 400), Size(200, 100), &operator>, ">", true))
			return TestFail;
		if (!compare(Size(100, 400), Size(200, 100), &operator>=, ">=", true))
			return TestFail;

		/* Test Size ordering based on width (with identical areas). */
		if (!compare(Size(100, 200), Size(200, 100), &operator<, "<", true))
			return TestFail;
		if (!compare(Size(100, 200), Size(200, 100), &operator<=, "<=", true))
			return TestFail;
		if (!compare(Size(100, 200), Size(200, 100), &operator>, ">", false))
			return TestFail;
		if (!compare(Size(100, 200), Size(200, 100), &operator>=, ">=", false))
			return TestFail;

		if (!compare(Size(200, 100), Size(100, 200), &operator<, "<", false))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 200), &operator<=, "<=", false))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 200), &operator>, ">", true))
			return TestFail;
		if (!compare(Size(200, 100), Size(100, 200), &operator>=, ">=", true))
			return TestFail;

		/*
		 * Rectangle tests
		 */

		/* Test Rectangle::isNull(). */
		if (!Rectangle(0, 0, 0, 0).isNull() ||
		    !Rectangle(1, 1, 0, 0).isNull()) {
			cout << "Null rectangle incorrectly reported as not null" << endl;
			return TestFail;
		}

		if (Rectangle(0, 0, 0, 1).isNull() ||
		    Rectangle(0, 0, 1, 0).isNull() ||
		    Rectangle(0, 0, 1, 1).isNull()) {
			cout << "Non-null rectangle incorrectly reported as null" << endl;
			return TestFail;
		}

		/* Rectangle::size(), Rectangle::topLeft() and Rectangle::center() tests */
		if (Rectangle(-1, -2, 3, 4).size() != Size(3, 4) ||
		    Rectangle(0, 0, 100000, 200000).size() != Size(100000, 200000)) {
			cout << "Rectangle::size() test failed" << endl;
			return TestFail;
		}

		if (Rectangle(1, 2, 3, 4).topLeft() != Point(1, 2) ||
		    Rectangle(-1, -2, 3, 4).topLeft() != Point(-1, -2)) {
			cout << "Rectangle::topLeft() test failed" << endl;
			return TestFail;
		}

		if (Rectangle(0, 0, 300, 400).center() != Point(150, 200) ||
		    Rectangle(-1000, -2000, 300, 400).center() != Point(-850, -1800) ||
		    Rectangle(10, 20, 301, 401).center() != Point(160, 220) ||
		    Rectangle(11, 21, 301, 401).center() != Point(161, 221) ||
		    Rectangle(-1011, -2021, 301, 401).center() != Point(-861, -1821)) {
			cout << "Rectangle::center() test failed" << endl;
			return TestFail;
		}

		/* Rectangle::boundedTo() (intersection function) */
		if (Rectangle(0, 0, 1000, 2000).boundedTo(Rectangle(0, 0, 1000, 2000)) !=
			    Rectangle(0, 0, 1000, 2000) ||
		    Rectangle(-500, -1000, 1000, 2000).boundedTo(Rectangle(0, 0, 1000, 2000)) !=
			    Rectangle(0, 0, 500, 1000) ||
		    Rectangle(500, 1000, 1000, 2000).boundedTo(Rectangle(0, 0, 1000, 2000)) !=
			    Rectangle(500, 1000, 500, 1000) ||
		    Rectangle(300, 400, 50, 100).boundedTo(Rectangle(0, 0, 1000, 2000)) !=
			    Rectangle(300, 400, 50, 100) ||
		    Rectangle(0, 0, 1000, 2000).boundedTo(Rectangle(300, 400, 50, 100)) !=
			    Rectangle(300, 400, 50, 100) ||
		    Rectangle(0, 0, 100, 100).boundedTo(Rectangle(50, 100, 100, 100)) !=
			    Rectangle(50, 100, 50, 0) ||
		    Rectangle(0, 0, 100, 100).boundedTo(Rectangle(100, 50, 100, 100)) !=
			    Rectangle(100, 50, 0, 50) ||
		    Rectangle(-10, -20, 10, 20).boundedTo(Rectangle(10, 20, 100, 100)) !=
			    Rectangle(10, 20, 0, 0)) {
			cout << "Rectangle::boundedTo() test failed" << endl;
			return TestFail;
		}

		/* Rectangle::enclosedIn() tests */
		if (Rectangle(10, 20, 300, 400).enclosedIn(Rectangle(-10, -20, 1300, 1400)) !=
			    Rectangle(10, 20, 300, 400) ||
		    Rectangle(-100, -200, 3000, 4000).enclosedIn(Rectangle(-10, -20, 1300, 1400)) !=
			    Rectangle(-10, -20, 1300, 1400) ||
		    Rectangle(-100, -200, 300, 400).enclosedIn(Rectangle(-10, -20, 1300, 1400)) !=
			    Rectangle(-10, -20, 300, 400) ||
		    Rectangle(5100, 6200, 300, 400).enclosedIn(Rectangle(-10, -20, 1300, 1400)) !=
			    Rectangle(990, 980, 300, 400) ||
		    Rectangle(100, -300, 150, 200).enclosedIn(Rectangle(50, 0, 200, 300)) !=
			    Rectangle(100, 0, 150, 200) ||
		    Rectangle(100, -300, 150, 1200).enclosedIn(Rectangle(50, 0, 200, 300)) !=
			    Rectangle(100, 0, 150, 300) ||
		    Rectangle(-300, 100, 200, 150).enclosedIn(Rectangle(0, 50, 300, 200)) !=
			    Rectangle(0, 100, 200, 150) ||
		    Rectangle(-300, 100, 1200, 150).enclosedIn(Rectangle(0, 50, 300, 200)) !=
			    Rectangle(0, 100, 300, 150)) {
			cout << "Rectangle::enclosedIn() test failed" << endl;
			return TestFail;
		}

		/* Rectange::scaledBy() tests */
		if (Rectangle(10, 20, 300, 400).scaledBy(Size(0, 0), Size(1, 1)) !=
			    Rectangle(0, 0, 0, 0) ||
		    Rectangle(10, -20, 300, 400).scaledBy(Size(32768, 65536), Size(32768, 32768)) !=
			    Rectangle(10, -40, 300, 800) ||
		    Rectangle(-30000, 10000, 20000, 20000).scaledBy(Size(7, 7), Size(7, 7)) !=
			    Rectangle(-30000, 10000, 20000, 20000) ||
		    Rectangle(-20, -30, 320, 240).scaledBy(Size(1280, 960), Size(640, 480)) !=
			    Rectangle(-40, -60, 640, 480) ||
		    Rectangle(1, 1, 2026, 1510).scaledBy(Size(4056, 3024), Size(2028, 1512)) !=
			    Rectangle(2, 2, 4052, 3020)) {
			cout << "Rectangle::scaledBy() test failed" << endl;
			return TestFail;
		}

		/* Rectangle::translatedBy() tests */
		if (Rectangle(10, -20, 300, 400).translatedBy(Point(-30, 40)) !=
			    Rectangle(-20, 20, 300, 400) ||
		    Rectangle(-10, 20, 400, 300).translatedBy(Point(50, -60)) !=
			    Rectangle(40, -40, 400, 300)) {
			cout << "Rectangle::translatedBy() test failed" << endl;
			return TestFail;
		}

		/* Rectangle::scaleBy() tests */
		Rectangle r(-20, -30, 320, 240);
		r.scaleBy(Size(1280, 960), Size(640, 480));
		if (r != Rectangle(-40, -60, 640, 480)) {
			cout << "Rectangle::scaleBy() test failed" << endl;
			return TestFail;
		}

		r = Rectangle(1, 1, 2026, 1510);
		r.scaleBy(Size(4056, 3024), Size(2028, 1512));
		if (r != Rectangle(2, 2, 4052, 3020)) {
			cout << "Rectangle::scaleBy() test failed" << endl;
			return TestFail;
		}

		/* Rectangle::translateBy() tests */
		r = Rectangle(10, -20, 300, 400);
		r.translateBy(Point(-30, 40));
		if (r != Rectangle(-20, 20, 300, 400)) {
			cout << "Rectangle::translateBy() test failed" << endl;
			return TestFail;
		}

		r = Rectangle(-10, 20, 400, 300);
		r.translateBy(Point(50, -60));
		if (r != Rectangle(40, -40, 400, 300)) {
			cout << "Rectangle::translateBy() test failed" << endl;
			return TestFail;
		}

		Point topLeft(3, 3);
		Point bottomRight(30, 30);
		Point topRight(30, 3);
		Point bottomLeft(3, 30);
		Rectangle rect1(topLeft, bottomRight);
		Rectangle rect2(topRight, bottomLeft);
		Rectangle rect3(bottomRight, topLeft);
		Rectangle rect4(bottomLeft, topRight);

		if (rect1 != rect2 || rect1 != rect3 || rect1 != rect4) {
			cout << "Point-to-point construction failed" << endl;
			return TestFail;
		}

		Rectangle f1 = Rectangle(100, 200, 3000, 2000);
		Rectangle f2 = Rectangle(200, 300, 1500, 1000);
		/* Bottom right quarter of the corresponding frames. */
		Rectangle r1 = Rectangle(100 + 1500, 200 + 1000, 1500, 1000);
		Rectangle r2 = Rectangle(200 + 750, 300 + 500, 750, 500);
		if (r1.transformedBetween(f1, f2) != r2 ||
		    r2.transformedBetween(f2, f1) != r1) {
			cout << "Rectangle::transformedBetween() test failed" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(GeometryTest)
