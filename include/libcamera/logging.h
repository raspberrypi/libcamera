/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * logging.h - Logging infrastructure
 */

#pragma once

namespace libcamera {

enum LoggingTarget {
	LoggingTargetNone,
	LoggingTargetSyslog,
	LoggingTargetFile,
	LoggingTargetStream,
};

int logSetFile(const char *path, bool color = false);
int logSetStream(std::ostream *stream, bool color = false);
int logSetTarget(LoggingTarget target);
void logSetLevel(const char *category, const char *level);

} /* namespace libcamera */
