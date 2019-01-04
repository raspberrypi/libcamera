/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * log.cpp - Logging infrastructure
 */

#include <cstdio>
#include <cstdlib>
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
 * \var LogDebug
 * Debug message
 * \var LogInfo
 * Informational message
 * \var LogWarning
 * Warning message, signals a potential issue
 * \var LogError
 * Error message, signals an unrecoverable issue
 * \var LogFatal
 * Fatal message, signals an unrecoverable issue and aborts execution
 */

/**
 * \def LOG(severity)
 * \brief Log a message
 *
 * Return an std::ostream reference to which a message can be logged using the
 * iostream API. The \a severity controls whether the message is printed or
 * dropped, depending on the global log level.
 *
 * If the severity is set to Fatal, execution is aborted and the program
 * terminates immediately after printing the message.
 */

static const char *log_severity_name(LogSeverity severity)
{
	static const char * const names[] = {
		"  DBG",
		" INFO",
		" WARN",
		"  ERR",
		"FATAL",
	};

	if (static_cast<unsigned int>(severity) < ARRAY_SIZE(names))
		return names[severity];
	else
		return "UNKN";
}

/**
 * \class LogMessage
 * \brief Internal log message representation.
 *
 * The LogMessage class models a single message in the log. It serves as a
 * helper to provide the std::ostream API for logging, and must never be used
 * directly. Use the LOG() macro instead access the log infrastructure.
 */

/**
 * Create a log message pertaining to line \a line of file \a fileName. The
 * \a severity argument sets the message severity to control whether it will be
 * output or dropped.
 */
LogMessage::LogMessage(const char *fileName, unsigned int line,
		       LogSeverity severity)
	: severity_(severity)
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

	if (severity_ == LogSeverity::LogFatal)
		std::abort();
}

/**
 * \fn std::ostream& LogMessage::stream()
 *
 * Data is added to a LogMessage through the stream returned by this function.
 * The stream implements the std::ostream API and can be used for logging as
 * std::cout.
 */

} /* namespace libcamera */
