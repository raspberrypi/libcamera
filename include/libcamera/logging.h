/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * logging.h - Logging infrastructure
 */
#ifndef __LIBCAMERA_LOGGING_H__
#define __LIBCAMERA_LOGGING_H__

namespace libcamera {

void logSetFile(const char *file);
int logSetLevel(const char *category, const char *level);

} /* namespace libcamera */

#endif /* __LIBCAMERA_LOGGING_H__ */
