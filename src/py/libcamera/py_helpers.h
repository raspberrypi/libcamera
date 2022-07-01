/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#pragma once

#include <libcamera/libcamera.h>

#include <pybind11/smart_holder.h>

pybind11::object controlValueToPy(const libcamera::ControlValue &cv);
libcamera::ControlValue pyToControlValue(const pybind11::object &ob, libcamera::ControlType type);
