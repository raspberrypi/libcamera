/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * log.h - Logging infrastructure
 */
#ifndef __LIBCAMERA_LOG_H__
#define __LIBCAMERA_LOG_H__

#include <sstream>

namespace libcamera {

enum LogSeverity {
	LogDebug,
	LogInfo,
	LogWarning,
	LogError,
	LogFatal,
};

class LogMessage
{
public:
	LogMessage(const char *fileName, unsigned int line,
		  LogSeverity severity);
	LogMessage(const LogMessage&) = delete;
	~LogMessage();

	std::ostream& stream() { return msgStream; }

private:
	std::ostringstream msgStream;
	LogSeverity severity_;
};

#define LOG(severity) LogMessage(__FILE__, __LINE__, Log##severity).stream()

} /* namespace libcamera */

#endif /* __LIBCAMERA_LOG_H__ */
