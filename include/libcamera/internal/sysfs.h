/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * sysfs.h - Miscellaneous utility functions to access sysfs
 */
#ifndef __LIBCAMERA_INTERNAL_SYSFS_H__
#define __LIBCAMERA_INTERNAL_SYSFS_H__

#include <string>

namespace libcamera {

namespace sysfs {

std::string charDevPath(const std::string &deviceNode);

std::string firmwareNodePath(const std::string &device);

} /* namespace sysfs */

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_SYSFS_H__ */
