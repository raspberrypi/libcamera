/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Geometry classes
 */

#include <array>

#include <libcamera/geometry.h>
#include <libcamera/libcamera.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "py_main.h"

namespace py = pybind11;

using namespace libcamera;

void init_py_geometry(py::module &m)
{
	auto pyPoint = py::class_<Point>(m, "Point");
	auto pySize = py::class_<Size>(m, "Size");
	auto pySizeRange = py::class_<SizeRange>(m, "SizeRange");
	auto pyRectangle = py::class_<Rectangle>(m, "Rectangle");

	pyPoint
		.def(py::init<>())
		.def(py::init<int, int>())
		.def_readwrite("x", &Point::x)
		.def_readwrite("y", &Point::y)
		.def(py::self == py::self)
		.def(-py::self)
		.def("__str__", &Point::toString)
		.def("__repr__", [](const Point &self) {
			return py::str("libcamera.Point({}, {})")
				.format(self.x, self.y);
		});

	pySize
		.def(py::init<>())
		.def(py::init<unsigned int, unsigned int>())
		.def_readwrite("width", &Size::width)
		.def_readwrite("height", &Size::height)
		.def_property_readonly("is_null", &Size::isNull)
		.def("align_down_to", &Size::alignDownTo)
		.def("align_up_to", &Size::alignUpTo)
		.def("bound_to", &Size::boundTo)
		.def("expand_to", &Size::expandTo)
		.def("grow_by", &Size::growBy)
		.def("shrink_by", &Size::shrinkBy)
		.def("aligned_up_to", &Size::alignedUpTo)
		.def("aligned_up_to", &Size::alignedUpTo)
		.def("bounded_to", &Size::boundedTo)
		.def("expanded_to", &Size::expandedTo)
		.def("grown_by", &Size::grownBy)
		.def("shrunk_by", &Size::shrunkBy)
		.def("bounded_to_aspect_ratio", &Size::boundedToAspectRatio)
		.def("expanded_to_aspect_ratio", &Size::expandedToAspectRatio)
		.def("centered_to", &Size::centeredTo)
		.def(py::self == py::self)
		.def(py::self < py::self)
		.def(py::self <= py::self)
		.def(py::self * float())
		.def(py::self / float())
		.def(py::self *= float())
		.def(py::self /= float())
		.def("__str__", &Size::toString)
		.def("__repr__", [](const Size &self) {
			return py::str("libcamera.Size({}, {})")
				.format(self.width, self.height);
		});

	pySizeRange
		.def(py::init<>())
		.def(py::init<Size>())
		.def(py::init<Size, Size>())
		.def(py::init<Size, Size, unsigned int, unsigned int>())
		.def_readwrite("min", &SizeRange::min)
		.def_readwrite("max", &SizeRange::max)
		.def_readwrite("hStep", &SizeRange::hStep)
		.def_readwrite("vStep", &SizeRange::vStep)
		.def("contains", &SizeRange::contains)
		.def(py::self == py::self)
		.def("__str__", &SizeRange::toString)
		.def("__repr__", [](const SizeRange &self) {
			return py::str("libcamera.SizeRange(({}, {}), ({}, {}), {}, {})")
				.format(self.min.width, self.min.height,
					self.max.width, self.max.height,
					self.hStep, self.vStep);
		});

	pyRectangle
		.def(py::init<>())
		.def(py::init<int, int, Size>())
		.def(py::init<int, int, unsigned int, unsigned int>())
		.def(py::init<Size>())
		.def_readwrite("x", &Rectangle::x)
		.def_readwrite("y", &Rectangle::y)
		.def_readwrite("width", &Rectangle::width)
		.def_readwrite("height", &Rectangle::height)
		.def_property_readonly("is_null", &Rectangle::isNull)
		.def_property_readonly("center", &Rectangle::center)
		.def_property_readonly("size", &Rectangle::size)
		.def_property_readonly("topLeft", &Rectangle::topLeft)
		.def("scale_by", &Rectangle::scaleBy)
		.def("translate_by", &Rectangle::translateBy)
		.def("bounded_to", &Rectangle::boundedTo)
		.def("enclosed_in", &Rectangle::enclosedIn)
		.def("scaled_by", &Rectangle::scaledBy)
		.def("translated_by", &Rectangle::translatedBy)
		.def(py::self == py::self)
		.def("__str__", &Rectangle::toString)
		.def("__repr__", [](const Rectangle &self) {
			return py::str("libcamera.Rectangle({}, {}, {}, {})")
				.format(self.x, self.y, self.width, self.height);
		});
}
