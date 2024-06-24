/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Color Space classes
 */

#include <libcamera/color_space.h>
#include <libcamera/libcamera.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "py_main.h"

namespace py = pybind11;

using namespace libcamera;

void init_py_color_space(py::module &m)
{
	auto pyColorSpace = py::class_<ColorSpace>(m, "ColorSpace");
	auto pyColorSpacePrimaries = py::enum_<ColorSpace::Primaries>(pyColorSpace, "Primaries");
	auto pyColorSpaceTransferFunction = py::enum_<ColorSpace::TransferFunction>(pyColorSpace, "TransferFunction");
	auto pyColorSpaceYcbcrEncoding = py::enum_<ColorSpace::YcbcrEncoding>(pyColorSpace, "YcbcrEncoding");
	auto pyColorSpaceRange = py::enum_<ColorSpace::Range>(pyColorSpace, "Range");

	pyColorSpace
		.def(py::init([](ColorSpace::Primaries primaries,
				 ColorSpace::TransferFunction transferFunction,
				 ColorSpace::YcbcrEncoding ycbcrEncoding,
				 ColorSpace::Range range) {
			return ColorSpace(primaries, transferFunction, ycbcrEncoding, range);
		}), py::arg("primaries"), py::arg("transferFunction"),
		    py::arg("ycbcrEncoding"), py::arg("range"))
		.def(py::init([](ColorSpace &other) { return other; }))
		.def("__str__", [](ColorSpace &self) {
			return "<libcamera.ColorSpace '" + self.toString() + "'>";
		})
		.def_readwrite("primaries", &ColorSpace::primaries)
		.def_readwrite("transferFunction", &ColorSpace::transferFunction)
		.def_readwrite("ycbcrEncoding", &ColorSpace::ycbcrEncoding)
		.def_readwrite("range", &ColorSpace::range)
		.def_static("Raw", []() { return ColorSpace::Raw; })
		.def_static("Srgb", []() { return ColorSpace::Srgb; })
		.def_static("Sycc", []() { return ColorSpace::Sycc; })
		.def_static("Smpte170m", []() { return ColorSpace::Smpte170m; })
		.def_static("Rec709", []() { return ColorSpace::Rec709; })
		.def_static("Rec2020", []() { return ColorSpace::Rec2020; });

	pyColorSpacePrimaries
		.value("Raw", ColorSpace::Primaries::Raw)
		.value("Smpte170m", ColorSpace::Primaries::Smpte170m)
		.value("Rec709", ColorSpace::Primaries::Rec709)
		.value("Rec2020", ColorSpace::Primaries::Rec2020);

	pyColorSpaceTransferFunction
		.value("Linear", ColorSpace::TransferFunction::Linear)
		.value("Srgb", ColorSpace::TransferFunction::Srgb)
		.value("Rec709", ColorSpace::TransferFunction::Rec709);

	pyColorSpaceYcbcrEncoding
		.value("Null", ColorSpace::YcbcrEncoding::None)
		.value("Rec601", ColorSpace::YcbcrEncoding::Rec601)
		.value("Rec709", ColorSpace::YcbcrEncoding::Rec709)
		.value("Rec2020", ColorSpace::YcbcrEncoding::Rec2020);

	pyColorSpaceRange
		.value("Full", ColorSpace::Range::Full)
		.value("Limited", ColorSpace::Range::Limited);
}
