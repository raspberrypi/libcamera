/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include "py_helpers.h"

#include <libcamera/libcamera.h>

#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

using namespace libcamera;

template<typename T>
static py::object valueOrTuple(const ControlValue &cv)
{
	if (cv.isArray()) {
		const T *v = reinterpret_cast<const T *>(cv.data().data());
		auto t = py::tuple(cv.numElements());

		for (size_t i = 0; i < cv.numElements(); ++i)
			t[i] = v[i];

		return std::move(t);
	}

	return py::cast(cv.get<T>());
}

py::object controlValueToPy(const ControlValue &cv)
{
	switch (cv.type()) {
	case ControlTypeBool:
		return valueOrTuple<bool>(cv);
	case ControlTypeByte:
		return valueOrTuple<uint8_t>(cv);
	case ControlTypeInteger32:
		return valueOrTuple<int32_t>(cv);
	case ControlTypeInteger64:
		return valueOrTuple<int64_t>(cv);
	case ControlTypeFloat:
		return valueOrTuple<float>(cv);
	case ControlTypeString:
		return py::cast(cv.get<std::string>());
	case ControlTypeRectangle: {
		const Rectangle *v = reinterpret_cast<const Rectangle *>(cv.data().data());
		return py::cast(v);
	}
	case ControlTypeSize: {
		const Size *v = reinterpret_cast<const Size *>(cv.data().data());
		return py::cast(v);
	}
	case ControlTypeNone:
		return py::none();
	default:
		throw std::runtime_error("Unsupported ControlValue type");
	}
}

template<typename T>
static ControlValue controlValueMaybeArray(const py::object &ob)
{
	if (py::isinstance<py::list>(ob) || py::isinstance<py::tuple>(ob)) {
		std::vector<T> vec = ob.cast<std::vector<T>>();
		return ControlValue(Span<const T>(vec));
	}

	return ControlValue(ob.cast<T>());
}

ControlValue pyToControlValue(const py::object &ob, ControlType type)
{
	switch (type) {
	case ControlTypeBool:
		return ControlValue(ob.cast<bool>());
	case ControlTypeByte:
		return controlValueMaybeArray<uint8_t>(ob);
	case ControlTypeInteger32:
		return controlValueMaybeArray<int32_t>(ob);
	case ControlTypeInteger64:
		return controlValueMaybeArray<int64_t>(ob);
	case ControlTypeFloat:
		return controlValueMaybeArray<float>(ob);
	case ControlTypeString:
		return ControlValue(ob.cast<std::string>());
	case ControlTypeRectangle:
		return ControlValue(ob.cast<Rectangle>());
	case ControlTypeSize:
		return ControlValue(ob.cast<Size>());
	case ControlTypeNone:
		return ControlValue();
	default:
		throw std::runtime_error("Control type not implemented");
	}
}
