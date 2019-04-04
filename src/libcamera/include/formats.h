/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * formats.h - Libcamera image formats
 */

#ifndef __LIBCAMERA_FORMATS_H__
#define __LIBCAMERA_FORMATS_H__

#include <map>
#include <vector>

#include <libcamera/geometry.h>

namespace libcamera {

typedef std::map<unsigned int, std::vector<SizeRange>> FormatEnum;

} /* namespace libcamera */

#endif /* __LIBCAMERA_FORMATS_H__ */
