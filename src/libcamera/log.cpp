/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * log.h - Logging infrastructure
 */

#include <cstdio>
#include <ctime>
#include <iomanip>
#include <string.h>

#include "log.h"
#include "utils.h"

/**
 * \file log.h
 * \brief Logging infrastructure
 */

namespace libcamera {

/**
 * \enum LogSeverity
 * Log message severity
 * \var Info
 * Informational message
 * \var Warning
 * Warning message, signals a potential issue
 * \var Error
 * Error message, signals an unrecoverable issue
 */

/**
 * \def LOG(severity)
 * \brief Log a message
 *
 * Return an std::ostream reference to which a message can be logged using the
 * iostream API. The \a severity controls whether the message is printed or
 * dropped, depending on the global log level.
 */

static const char *log_severity_name(LogSeverity severity)
{
	static const char * const names[] = {
		"INFO",
		"WARN",
		" ERR",
	};

	if ((unsigned int)severity < ARRAY_SIZE(names))
		return names[severity];
	else
		return "UNKN";
}

LogMessage::LogMessage(const char *fileName, unsigned int line,
		       LogSeverity severity)
{
	/* Log the timestamp, severity and file information. */
	struct timespec timestamp;
	clock_gettime(CLOCK_MONOTONIC, &timestamp);
	msgStream << "[" << timestamp.tv_sec / (60 * 60) << ":"
		  << std::setw(2) << (timestamp.tv_sec / 60) % 60 << ":"
		  << std::setw(2) << timestamp.tv_sec % 60 << "."
		  << std::setw(9) << timestamp.tv_nsec << "]";

	msgStream << " " << log_severity_name(severity);
	msgStream << " " << basename(fileName) << ":" << line << " ";
}

LogMessage::~LogMessage()
{
	msgStream << std::endl;

	std::string msg(msgStream.str());
	fwrite(msg.data(), msg.size(), 1, stderr);
	fflush(stderr);
}

};
