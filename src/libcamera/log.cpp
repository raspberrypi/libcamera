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
 * \class LogCategory
 * \brief A category of log message
 *
 * The LogCategory class represents a category of log messages, related to an
 * area of the library. It groups all messages belonging to the same category,
 * and is used to control the log level per group.
 */

/**
 * \brief Construct a log category
 * \param[in] name The category name
 */
LogCategory::LogCategory(const char *name)
	: name_(name), severity_(LogSeverity::LogInfo)
{
}

LogCategory::~LogCategory()
{
}

/**
 * \fn LogCategory::name()
 * \brief Retrieve the log category name
 * \return The log category name
 */

/**
 * \fn LogCategory::severity()
 * \brief Retrieve the severity of the log category
 * \sa setSeverity()
 * \return Return the severity of the log category
 */

/**
 * \brief Set the severity of the log category
 *
 * Messages of severity higher than or equal to the severity of the log category
 * are printed, other messages are discarded.
 */
void LogCategory::setSeverity(LogSeverity severity)
{
	severity_ = severity;
}

/**
 * \brief Retrieve the default log category
 *
 * The default log category is named "default" and is used by the LOG() macro
 * when no log category is specified.
 *
 * \return A pointer to the default log category
 */
const LogCategory &LogCategory::defaultCategory()
{
	static LogCategory category("default");
	return category;
}

static const char *log_severity_name(LogSeverity severity)
{
	static const char *const names[] = {
		"  DBG",
		" INFO",
		" WARN",
		"  ERR",
		"FATAL",
	};

	if (static_cast<unsigned int>(severity) < ARRAY_SIZE(names))
		return names[severity];
	else
		return "UNKWN";
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
 * \brief Construct a log message for the default category
 * \param[in] fileName The file name where the message is logged from
 * \param[in] line The line number where the message is logged from
 * \param[in] severity The log message severity, controlling how the message
 * will be displayed
 *
 * Create a log message pertaining to line \a line of file \a fileName. The
 * \a severity argument sets the message severity to control whether it will be
 * output or dropped.
 */
LogMessage::LogMessage(const char *fileName, unsigned int line,
		       LogSeverity severity)
	: category_(LogCategory::defaultCategory()), severity_(severity)
{
	init(fileName, line);
}

/**
 * \brief Construct a log message for a given category
 * \param[in] fileName The file name where the message is logged from
 * \param[in] line The line number where the message is logged from
 * \param[in] category The log message category, controlling how the message
 * will be displayed
 * \param[in] severity The log message severity, controlling how the message
 * will be displayed
 *
 * Create a log message pertaining to line \a line of file \a fileName. The
 * \a severity argument sets the message severity to control whether it will be
 * output or dropped.
 */
LogMessage::LogMessage(const char *fileName, unsigned int line,
		       const LogCategory &category, LogSeverity severity)
	: category_(category), severity_(severity)
{
	init(fileName, line);
}

void LogMessage::init(const char *fileName, unsigned int line)
{
	/* Log the timestamp, severity and file information. */
	struct timespec timestamp;
	clock_gettime(CLOCK_MONOTONIC, &timestamp);
	msgStream_.fill('0');
	msgStream_ << "[" << timestamp.tv_sec / (60 * 60) << ":"
		   << std::setw(2) << (timestamp.tv_sec / 60) % 60 << ":"
		   << std::setw(2) << timestamp.tv_sec % 60 << "."
		   << std::setw(9) << timestamp.tv_nsec << "]";
	msgStream_.fill(' ');

	msgStream_ << " " << log_severity_name(severity_);
	msgStream_ << " " << category_.name();
	msgStream_ << " " << basename(fileName) << ":" << line << " ";
}

LogMessage::~LogMessage()
{
	msgStream_ << std::endl;

	if (severity_ >= category_.severity()) {
		std::string msg(msgStream_.str());
		fwrite(msg.data(), msg.size(), 1, stderr);
		fflush(stderr);
	}

	if (severity_ == LogSeverity::LogFatal)
		std::abort();
}

/**
 * \fn std::ostream& LogMessage::stream()
 *
 * Data is added to a LogMessage through the stream returned by this function.
 * The stream implements the std::ostream API and can be used for logging as
 * std::cout.
 *
 * \return A reference to the log message stream
 */

/**
 * \def LOG_DECLARE_CATEGORY(name)
 * \hideinitializer
 * \brief Declare a category of log messages
 *
 * This macro is used to declare a log category defined in another compilation
 * unit by the LOG_DEFINE_CATEGORY() macro.
 *
 * The LOG_DECLARE_CATEGORY() macro must be used in the libcamera namespace.
 *
 * \sa LogCategory
 */

/**
 * \def LOG_DEFINE_CATEGORY(name)
 * \hideinitializer
 * \brief Define a category of log messages
 *
 * This macro is used to define a log category that can then be used with the
 * LOGC() macro. Category names shall be unique, if a category is shared between
 * compilation units, it shall be defined in one compilation unit only and
 * declared with LOG_DECLARE_CATEGORY() in the other compilation units.
 *
 * The LOG_DEFINE_CATEGORY() macro must be used in the libcamera namespace.
 *
 * \sa LogCategory
 */

/**
 * \def LOG(category, severity)
 * \hideinitializer
 * \brief Log a message
 * \param[in] category Category (optional)
 * \param[in] severity Severity
 *
 * Return an std::ostream reference to which a message can be logged using the
 * iostream API. The \a category, if specified, sets the message category. When
 * absent the default category is used. The  \a severity controls whether the
 * message is printed or discarded, depending on the log level for the category.
 *
 * If the severity is set to Fatal, execution is aborted and the program
 * terminates immediately after printing the message.
 */

/**
 * \def ASSERT(condition)
 * \brief Abort program execution if assertion fails
 *
 * If \a condition is false, ASSERT() logs an error message with the Fatal log
 * level and aborts program execution.
 *
 * If the macro NDEBUG is defined before including log.h, ASSERT() generates no
 * code.
 *
 * Using conditions that have side effects with ASSERT() is not recommended, as
 * these effects would depend on whether NDEBUG is defined or not. Similarly,
 * ASSERT() should not be used to check for errors that can occur under normal
 * conditions as those checks would then be removed when compiling with NDEBUG.
 */

} /* namespace libcamera */
