/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Enumerations
 */

#include <libcamera/libcamera.h>

#include <pybind11/pybind11.h>

#include "py_main.h"

namespace py = pybind11;

using namespace libcamera;

void init_py_enums(py::module &m)
{
	py::enum_<StreamRole>(m, "StreamRole")
		.value("StillCapture", StreamRole::StillCapture)
		.value("Raw", StreamRole::Raw)
		.value("VideoRecording", StreamRole::VideoRecording)
		.value("Viewfinder", StreamRole::Viewfinder);

	py::enum_<ControlType>(m, "ControlType")
		.value("Null", ControlType::ControlTypeNone)
		.value("Bool", ControlType::ControlTypeBool)
		.value("Byte", ControlType::ControlTypeByte)
		.value("Integer32", ControlType::ControlTypeInteger32)
		.value("Integer64", ControlType::ControlTypeInteger64)
		.value("Float", ControlType::ControlTypeFloat)
		.value("String", ControlType::ControlTypeString)
		.value("Rectangle", ControlType::ControlTypeRectangle)
		.value("Size", ControlType::ControlTypeSize)
		.value("Point", ControlType::ControlTypePoint);

	py::enum_<Orientation>(m, "Orientation")
		.value("Rotate0", Orientation::Rotate0)
		.value("Rotate0Mirror", Orientation::Rotate0Mirror)
		.value("Rotate180", Orientation::Rotate180)
		.value("Rotate180Mirror", Orientation::Rotate180Mirror)
		.value("Rotate90Mirror", Orientation::Rotate90Mirror)
		.value("Rotate270", Orientation::Rotate270)
		.value("Rotate270Mirror", Orientation::Rotate270Mirror)
		.value("Rotate90", Orientation::Rotate90);
}
