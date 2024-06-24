/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Transform class
 */

#include <libcamera/transform.h>
#include <libcamera/libcamera.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "py_main.h"

namespace py = pybind11;

using namespace libcamera;

void init_py_transform(py::module &m)
{
	auto pyTransform = py::class_<Transform>(m, "Transform");

	pyTransform
		.def(py::init([](int rotation, bool hflip, bool vflip, bool transpose) {
			bool ok;

			Transform t = transformFromRotation(rotation, &ok);
			if (!ok)
				throw std::invalid_argument("Invalid rotation");

			if (hflip)
				t ^= Transform::HFlip;
			if (vflip)
				t ^= Transform::VFlip;
			if (transpose)
				t ^= Transform::Transpose;
			return t;
		}), py::arg("rotation") = 0, py::arg("hflip") = false,
		    py::arg("vflip") = false, py::arg("transpose") = false)
		.def(py::init([](Transform &other) { return other; }))
		.def("__str__", [](Transform &self) {
			return "<libcamera.Transform '" + std::string(transformToString(self)) + "'>";
		})
		.def_property("hflip",
			      [](Transform &self) {
				      return !!(self & Transform::HFlip);
			      },
			      [](Transform &self, bool hflip) {
				      if (hflip)
					      self |= Transform::HFlip;
				      else
					      self &= ~Transform::HFlip;
			      })
		.def_property("vflip",
			      [](Transform &self) {
				      return !!(self & Transform::VFlip);
			      },
			      [](Transform &self, bool vflip) {
				      if (vflip)
					      self |= Transform::VFlip;
				      else
					      self &= ~Transform::VFlip;
			      })
		.def_property("transpose",
			      [](Transform &self) {
				      return !!(self & Transform::Transpose);
			      },
			      [](Transform &self, bool transpose) {
				      if (transpose)
					      self |= Transform::Transpose;
				      else
					      self &= ~Transform::Transpose;
			      })
		.def("inverse", [](Transform &self) { return -self; })
		.def("invert", [](Transform &self) {
			self = -self;
		})
		.def("compose", [](Transform &self, Transform &other) {
			self = self * other;
		});
}
