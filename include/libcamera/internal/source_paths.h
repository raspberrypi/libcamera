/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Identify libcamera source and build paths
 */

#pragma once

#include <string>

namespace libcamera::utils {

std::string libcameraBuildPath();
std::string libcameraSourcePath();

} /* namespace libcamera::utils */
