/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#pragma once

#include <pybind11/pybind11.h>

#include <libcamera/libcamera.h>

pybind11::object controlValueToPy(const libcamera::ControlValue &cv);
libcamera::ControlValue pyToControlValue(const pybind11::object &ob, libcamera::ControlType type);
