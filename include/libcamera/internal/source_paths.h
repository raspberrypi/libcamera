/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * source_paths.h - Identify libcamera source and build paths
 */
#ifndef __LIBCAMERA_INTERNAL_SOURCE_PATHS_H__
#define __LIBCAMERA_INTERNAL_SOURCE_PATHS_H__

#include <string>

namespace libcamera::utils {

std::string libcameraBuildPath();
std::string libcameraSourcePath();

} /* namespace libcamera::utils */

#endif /* __LIBCAMERA_INTERNAL_SOURCE_PATHS_H__ */
