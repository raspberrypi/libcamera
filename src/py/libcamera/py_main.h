/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#pragma once

#include <libcamera/base/log.h>

#include <pybind11/pybind11.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(Python)

}

void init_py_color_space(pybind11::module &m);
void init_py_controls_generated(pybind11::module &m);
void init_py_enums(pybind11::module &m);
void init_py_formats_generated(pybind11::module &m);
void init_py_geometry(pybind11::module &m);
void init_py_properties_generated(pybind11::module &m);
void init_py_transform(pybind11::module &m);
