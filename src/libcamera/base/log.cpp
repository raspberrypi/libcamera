/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * log.cpp - Logging infrastructure
 */

#include <libcamera/base/log.h>

#include <array>
#include <fstream>
#include <iostream>
#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unordered_set>

#include <libcamera/logging.h>

#include <libcamera/base/backtrace.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/utils.h>

/**
 * \file base/log.h
 * \brief Logging infrastructure
 *
 * libcamera includes a logging infrastructure used through the library that
 * allows inspection of internal operation in a user-configurable way. The log
 * messages are grouped in categories that represent areas of libcamera, and
 * output of messages for each category can be controlled by independent log
 * levels.
 *
 * The levels are configurable through the LIBCAMERA_LOG_LEVELS environment
 * variable that contains a comma-separated list of 'category:level' pairs.
 *
 * The category names are strings and can include a wildcard ('*') character at
 * the end to match multiple categories.
 *
 * The level are either numeric values, or strings containing the log level
 * name. The available log levels are DEBUG, INFO, WARN, ERROR and FATAL. Log
 * message with a level higher than or equal to the configured log level for
 * their category are output to the log, while other messages are silently
 * discarded.
 *
 * By default log messages are output to std::cerr. They can be redirected to a
 * log file by setting the LIBCAMERA_LOG_FILE environment variable to the name
 * of the file. The file must be writable and is truncated if it exists. If any
 * error occurs when opening the file, the file is ignored and the log is output
 * to std::cerr.
 */

/**
 * \file logging.h
 * \brief Logging management
 *
 * API to change the logging output destination and log levels programatically.
 */

namespace libcamera {

static int log_severity_to_syslog(LogSeverity severity)
{
	switch (severity) {
	case LogDebug:
		return LOG_DEBUG;
	case LogInfo:
		return LOG_INFO;
	case LogWarning:
		return LOG_WARNING;
	case LogError:
		return LOG_ERR;
	case LogFatal:
		return LOG_ALERT;
	default:
		return LOG_NOTICE;
	}
}

static const char *log_severity_name(LogSeverity severity)
{
	static const char *const names[] = {
		"DEBUG",
		" INFO",
		" WARN",
		"ERROR",
		"FATAL",
	};

	if (static_cast<unsigned int>(severity) < std::size(names))
		return names[severity];
	else
		return "UNKWN";
}

/**
 * \brief Log output
 *
 * The LogOutput class models a log output destination
 */
class LogOutput
{
public:
	LogOutput(const char *path, bool color);
	LogOutput(std::ostream *stream, bool color);
	LogOutput();
	~LogOutput();

	bool isValid() const;
	void write(const LogMessage &msg);
	void write(const std::string &msg);

private:
	void writeSyslog(LogSeverity severity, const std::string &msg);
	void writeStream(const std::string &msg);

	std::ostream *stream_;
	LoggingTarget target_;
	bool color_;
};

/**
 * \brief Construct a log output based on a file
 * \param[in] path Full path to log file
 * \param[in] color True to output colored messages
 */
LogOutput::LogOutput(const char *path, bool color)
	: target_(LoggingTargetFile), color_(color)
{
	stream_ = new std::ofstream(path);
}

/**
 * \brief Construct a log output based on a stream
 * \param[in] stream Stream to send log output to
 * \param[in] color True to output colored messages
 */
LogOutput::LogOutput(std::ostream *stream, bool color)
	: stream_(stream), target_(LoggingTargetStream), color_(color)
{
}

/**
 * \brief Construct a log output to syslog
 */
LogOutput::LogOutput()
	: stream_(nullptr), target_(LoggingTargetSyslog), color_(false)
{
	openlog("libcamera", LOG_PID, 0);
}

LogOutput::~LogOutput()
{
	switch (target_) {
	case LoggingTargetFile:
		delete stream_;
		break;
	case LoggingTargetSyslog:
		closelog();
		break;
	default:
		break;
	}
}

/**
 * \brief Check if the log output is valid
 * \return True if the log output is valid
 */
bool LogOutput::isValid() const
{
	switch (target_) {
	case LoggingTargetFile:
		return stream_->good();
	case LoggingTargetStream:
		return stream_ != nullptr;
	default:
		return true;
	}
}

namespace {

/*
 * For more information about ANSI escape codes, see
 * https://en.wikipedia.org/wiki/ANSI_escape_code#Colors.
 */
constexpr const char *kColorReset = "\033[0m";
constexpr const char *kColorGreen = "\033[0;32m";
constexpr const char *kColorBrightRed = "\033[1;31m";
constexpr const char *kColorBrightGreen = "\033[1;32m";
constexpr const char *kColorBrightYellow = "\033[1;33m";
constexpr const char *kColorBrightBlue = "\033[1;34m";
constexpr const char *kColorBrightMagenta = "\033[1;35m";
constexpr const char *kColorBrightCyan = "\033[1;36m";
constexpr const char *kColorBrightWhite = "\033[1;37m";

} /* namespace */

/**
 * \brief Write message to log output
 * \param[in] msg Message to write
 */
void LogOutput::write(const LogMessage &msg)
{
	static const char *const severityColors[] = {
		kColorBrightCyan,
		kColorBrightGreen,
		kColorBrightYellow,
		kColorBrightRed,
		kColorBrightMagenta,
	};

	const char *categoryColor = color_ ? kColorBrightWhite : "";
	const char *fileColor = color_ ? kColorBrightBlue : "";
	const char *prefixColor = color_ ? kColorGreen : "";
	const char *resetColor = color_ ? kColorReset : "";
	const char *severityColor = "";
	LogSeverity severity = msg.severity();
	std::string str;

	if (color_) {
		if (static_cast<unsigned int>(severity) < std::size(severityColors))
			severityColor = severityColors[severity];
		else
			severityColor = kColorBrightWhite;
	}

	switch (target_) {
	case LoggingTargetSyslog:
		str = std::string(log_severity_name(severity)) + " "
		    + msg.category().name() + " " + msg.fileInfo() + " ";
		if (!msg.prefix().empty())
			str += msg.prefix() + ": ";
		str += msg.msg();
		writeSyslog(severity, str);
		break;
	case LoggingTargetStream:
	case LoggingTargetFile:
		str = "[" + utils::time_point_to_string(msg.timestamp()) + "] ["
		    + std::to_string(Thread::currentId()) + "] "
		    + severityColor + log_severity_name(severity) + " "
		    + categoryColor + msg.category().name() + " "
		    + fileColor + msg.fileInfo() + " ";
		if (!msg.prefix().empty())
			str += prefixColor + msg.prefix() + ": ";
		str += resetColor + msg.msg();
		writeStream(str);
		break;
	default:
		break;
	}
}

/**
 * \brief Write string to log output
 * \param[in] str String to write
 */
void LogOutput::write(const std::string &str)
{
	switch (target_) {
	case LoggingTargetSyslog:
		writeSyslog(LogDebug, str);
		break;
	case LoggingTargetStream:
	case LoggingTargetFile:
		writeStream(str);
		break;
	default:
		break;
	}
}

void LogOutput::writeSyslog(LogSeverity severity, const std::string &str)
{
	syslog(log_severity_to_syslog(severity), "%s", str.c_str());
}

void LogOutput::writeStream(const std::string &str)
{
	stream_->write(str.c_str(), str.size());
	stream_->flush();
}

/**
 * \brief Message logger
 *
 * The Logger class handles log configuration.
 */
class Logger
{
public:
	~Logger();

	static Logger *instance();

	void write(const LogMessage &msg);
	void backtrace();

	int logSetFile(const char *path, bool color);
	int logSetStream(std::ostream *stream, bool color);
	int logSetTarget(LoggingTarget target);
	void logSetLevel(const char *category, const char *level);

private:
	Logger();

	void parseLogFile();
	void parseLogLevels();
	static LogSeverity parseLogLevel(const std::string &level);

	friend LogCategory;
	void registerCategory(LogCategory *category);
	LogCategory *findCategory(const char *name) const;

	static bool destroyed_;

	std::vector<LogCategory *> categories_;
	std::list<std::pair<std::string, LogSeverity>> levels_;

	std::shared_ptr<LogOutput> output_;
};

bool Logger::destroyed_ = false;

/**
 * \enum LoggingTarget
 * \brief Log destination type
 * \var LoggingTargetNone
 * \brief No logging destination
 * \sa Logger::logSetTarget
 * \var LoggingTargetSyslog
 * \brief Log to syslog
 * \sa Logger::logSetTarget
 * \var LoggingTargetFile
 * \brief Log to file
 * \sa Logger::logSetFile
 * \var LoggingTargetStream
 * \brief Log to stream
 * \sa Logger::logSetStream
 */

/**
 * \brief Direct logging to a file
 * \param[in] path Full path to the log file
 * \param[in] color True to output colored messages
 *
 * This function directs the log output to the file identified by \a path. The
 * previous log target, if any, is closed, and all new log messages will be
 * written to the new log file.
 *
 * \a color controls whether or not the messages will be colored with standard
 * ANSI escape codes. This is done regardless of whether \a path refers to a
 * standard file or a TTY, the caller is responsible for disabling coloring when
 * not suitable for the log target.
 *
 * If the function returns an error, the log target is not changed.
 *
 * \return Zero on success, or a negative error code otherwise
 */
int logSetFile(const char *path, bool color)
{
	return Logger::instance()->logSetFile(path, color);
}

/**
 * \brief Direct logging to a stream
 * \param[in] stream Stream to send log output to
 * \param[in] color True to output colored messages
 *
 * This function directs the log output to \a stream. The previous log target,
 * if any, is closed, and all new log messages will be written to the new log
 * stream.
 *
 * \a color controls whether or not the messages will be colored with standard
 * ANSI escape codes. This is done regardless of whether \a stream refers to a
 * standard file or a TTY, the caller is responsible for disabling coloring when
 * not suitable for the log target.
 *
 * If the function returns an error, the log file is not changed
 *
 * \return Zero on success, or a negative error code otherwise.
 */
int logSetStream(std::ostream *stream, bool color)
{
	return Logger::instance()->logSetStream(stream, color);
}

/**
 * \brief Set the logging target
 * \param[in] target Logging destination
 *
 * This function sets the logging output to the target specified by \a target.
 * The allowed values of \a target are LoggingTargetNone and
 * LoggingTargetSyslog. LoggingTargetNone will send the log output to nowhere,
 * and LoggingTargetSyslog will send the log output to syslog. The previous
 * log target, if any, is closed, and all new log messages will be written to
 * the new log destination.
 *
 * LoggingTargetFile and LoggingTargetStream are not valid values for \a target.
 * Use logSetFile() and logSetStream() instead, respectively.
 *
 * If the function returns an error, the log file is not changed.
 *
 * \return Zero on success, or a negative error code otherwise.
 */
int logSetTarget(LoggingTarget target)
{
	return Logger::instance()->logSetTarget(target);
}

/**
 * \brief Set the log level
 * \param[in] category Logging category
 * \param[in] level Log level
 *
 * This function sets the log level of \a category to \a level.
 * \a level shall be one of the following strings:
 * - "DEBUG"
 * - "INFO"
 * - "WARN"
 * - "ERROR"
 * - "FATAL"
 *
 * "*" is not a valid \a category for this function.
 */
void logSetLevel(const char *category, const char *level)
{
	Logger::instance()->logSetLevel(category, level);
}

Logger::~Logger()
{
	destroyed_ = true;

	for (LogCategory *category : categories_)
		delete category;
}

/**
 * \brief Retrieve the logger instance
 *
 * The Logger is a singleton and can't be constructed manually. This function
 * shall instead be used to retrieve the single global instance of the logger.
 *
 * \return The logger instance
 */
Logger *Logger::instance()
{
	static Logger instance;

	if (destroyed_)
		return nullptr;

	return &instance;
}

/**
 * \brief Write a message to the configured logger output
 * \param[in] msg The message object
 */
void Logger::write(const LogMessage &msg)
{
	std::shared_ptr<LogOutput> output = std::atomic_load(&output_);
	if (!output)
		return;

	output->write(msg);
}

/**
 * \brief Write a backtrace to the log
 */
void Logger::backtrace()
{
	std::shared_ptr<LogOutput> output = std::atomic_load(&output_);
	if (!output)
		return;

	/*
	 * Skip the first two entries that correspond to this function and
	 * ~LogMessage().
	 */
	std::string backtrace = Backtrace().toString(2);
	if (backtrace.empty()) {
		output->write("Backtrace not available\n");
		return;
	}

	output->write("Backtrace:\n");
	output->write(backtrace);
}

/**
 * \brief Set the log file
 * \param[in] path Full path to the log file
 * \param[in] color True to output colored messages
 *
 * \sa libcamera::logSetFile()
 *
 * \return Zero on success, or a negative error code otherwise.
 */
int Logger::logSetFile(const char *path, bool color)
{
	std::shared_ptr<LogOutput> output =
		std::make_shared<LogOutput>(path, color);
	if (!output->isValid())
		return -EINVAL;

	std::atomic_store(&output_, output);
	return 0;
}

/**
 * \brief Set the log stream
 * \param[in] stream Stream to send log output to
 * \param[in] color True to output colored messages
 *
 * \sa libcamera::logSetStream()
 *
 * \return Zero on success, or a negative error code otherwise.
 */
int Logger::logSetStream(std::ostream *stream, bool color)
{
	std::shared_ptr<LogOutput> output =
		std::make_shared<LogOutput>(stream, color);
	std::atomic_store(&output_, output);
	return 0;
}

/**
 * \brief Set the log target
 * \param[in] target Log destination
 *
 * \sa libcamera::logSetTarget()
 *
 * \return Zero on success, or a negative error code otherwise.
 */
int Logger::logSetTarget(enum LoggingTarget target)
{
	switch (target) {
	case LoggingTargetSyslog:
		std::atomic_store(&output_, std::make_shared<LogOutput>());
		break;
	case LoggingTargetNone:
		std::atomic_store(&output_, std::shared_ptr<LogOutput>());
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * \brief Set the log level
 * \param[in] category Logging category
 * \param[in] level Log level
 *
 * \sa libcamera::logSetLevel()
 */
void Logger::logSetLevel(const char *category, const char *level)
{
	LogSeverity severity = parseLogLevel(level);
	if (severity == LogInvalid)
		return;

	for (LogCategory *c : categories_) {
		if (c->name() == category) {
			c->setSeverity(severity);
			break;
		}
	}
}

/**
 * \brief Construct a logger
 *
 * If the environment variable is not set, log to std::cerr. The log messages
 * are then colored by default. This can be overridden by setting the
 * LIBCAMERA_LOG_NO_COLOR environment variable to disable coloring.
 */
Logger::Logger()
{
	bool color = !utils::secure_getenv("LIBCAMERA_LOG_NO_COLOR");
	logSetStream(&std::cerr, color);

	parseLogFile();
	parseLogLevels();
}

/**
 * \brief Parse the log output file from the environment
 *
 * If the LIBCAMERA_LOG_FILE environment variable is set, open the file it
 * points to and redirect the logger output to it. If the environment variable
 * is set to "syslog", then the logger output will be directed to syslog. Errors
 * are silently ignored and don't affect the logger output (set to std::cerr by
 * default).
 */
void Logger::parseLogFile()
{
	const char *file = utils::secure_getenv("LIBCAMERA_LOG_FILE");
	if (!file)
		return;

	if (!strcmp(file, "syslog")) {
		logSetTarget(LoggingTargetSyslog);
		return;
	}

	logSetFile(file, false);
}

/**
 * \brief Parse the log levels from the environment
 *
 * The log levels are stored in the LIBCAMERA_LOG_LEVELS environment variable
 * as a list of "category:level" pairs, separated by commas (','). Parse the
 * variable and store the levels to configure all log categories.
 */
void Logger::parseLogLevels()
{
	const char *debug = utils::secure_getenv("LIBCAMERA_LOG_LEVELS");
	if (!debug)
		return;

	for (const char *pair = debug; *debug != '\0'; pair = debug) {
		const char *comma = strchrnul(debug, ',');
		size_t len = comma - pair;

		/* Skip over the comma. */
		debug = *comma == ',' ? comma + 1 : comma;

		/* Skip to the next pair if the pair is empty. */
		if (!len)
			continue;

		std::string category;
		std::string level;

		const char *colon = static_cast<const char *>(memchr(pair, ':', len));
		if (!colon) {
			/* 'x' is a shortcut for '*:x'. */
			category = "*";
			level = std::string(pair, len);
		} else {
			category = std::string(pair, colon - pair);
			level = std::string(colon + 1, comma - colon - 1);
		}

		/* Both the category and the level must be specified. */
		if (category.empty() || level.empty())
			continue;

		LogSeverity severity = parseLogLevel(level);
		if (severity == LogInvalid)
			continue;

		levels_.push_back({ category, severity });
	}
}

/**
 * \brief Parse a log level string into a LogSeverity
 * \param[in] level The log level string
 *
 * Log levels can be specified as an integer value in the range from LogDebug to
 * LogFatal, or as a string corresponding to the severity name in uppercase. Any
 * other value is invalid.
 *
 * \return The log severity, or LogInvalid if the string is invalid
 */
LogSeverity Logger::parseLogLevel(const std::string &level)
{
	static const char *const names[] = {
		"DEBUG",
		"INFO",
		"WARN",
		"ERROR",
		"FATAL",
	};

	int severity;

	if (std::isdigit(level[0])) {
		char *endptr;
		severity = strtoul(level.c_str(), &endptr, 10);
		if (*endptr != '\0' || severity > LogFatal)
			severity = LogInvalid;
	} else {
		severity = LogInvalid;
		for (unsigned int i = 0; i < std::size(names); ++i) {
			if (names[i] == level) {
				severity = i;
				break;
			}
		}
	}

	return static_cast<LogSeverity>(severity);
}

/**
 * \brief Register a log category with the logger
 * \param[in] category The log category
 *
 * Log categories must have unique names. It is invalid to call this function
 * if a log category with the same name already exists.
 */
void Logger::registerCategory(LogCategory *category)
{
	categories_.push_back(category);

	const std::string &name = category->name();
	for (const std::pair<std::string, LogSeverity> &level : levels_) {
		bool match = true;

		for (unsigned int i = 0; i < level.first.size(); ++i) {
			if (level.first[i] == '*')
				break;

			if (i >= name.size() ||
			    name[i] != level.first[i]) {
				match = false;
				break;
			}
		}

		if (match) {
			category->setSeverity(level.second);
			break;
		}
	}
}

/**
 * \brief Find an existing log category with the given name
 * \param[in] name Name of the log category
 * \return The pointer to the found log category or nullptr if not found
 */
LogCategory *Logger::findCategory(const char *name) const
{
	if (auto it = std::find_if(categories_.begin(), categories_.end(),
				   [name](auto c) { return c->name() == name; });
	    it != categories_.end()) {
		return *it;
	}

	return nullptr;
}

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
 * \brief Create a new LogCategory or return an existing one
 * \param[in] name Name of the log category
 *
 * Create and return a new LogCategory with the given name if such a category
 * does not yet exist, or return the existing one.
 *
 * \return The pointer to the LogCategory
 */
LogCategory *LogCategory::create(const char *name)
{
	LogCategory *category = Logger::instance()->findCategory(name);

	if (!category) {
		category = new LogCategory(name);
		Logger::instance()->registerCategory(category);
	}

	return category;
}

/**
 * \brief Construct a log category
 * \param[in] name The category name
 */
LogCategory::LogCategory(const char *name)
	: name_(name), severity_(LogSeverity::LogInfo)
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
 * \return A reference to the default log category
 */
const LogCategory &LogCategory::defaultCategory()
{
	static const LogCategory *category = LogCategory::create("default");
	return *category;
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
 * \brief Construct a log message for a given category
 * \param[in] fileName The file name where the message is logged from
 * \param[in] line The line number where the message is logged from
 * \param[in] category The log message category, controlling how the message
 * will be displayed
 * \param[in] severity The log message severity, controlling how the message
 * will be displayed
 * \param[in] prefix The log message prefix
 *
 * Create a log message pertaining to line \a line of file \a fileName. The
 * \a severity argument sets the message severity to control whether it will be
 * output or dropped. The \a prefix optionally identifies the object instance
 * logging the message.
 */
LogMessage::LogMessage(const char *fileName, unsigned int line,
		       const LogCategory &category, LogSeverity severity,
		       const std::string &prefix)
	: category_(category), severity_(severity), prefix_(prefix)
{
	init(fileName, line);
}

/**
 * \brief Move-construct a log message
 * \param[in] other The other message
 *
 * The move constructor is meant to support the _log() functions. Thanks to copy
 * elision it will likely never be called, but C++11 only permits copy elision,
 * it doesn't enforce it unlike C++17. To avoid potential link errors depending
 * on the compiler type and version, and optimization level, the move
 * constructor is defined even if it will likely never be called, and ensures
 * that the destructor of the \a other message will not output anything to the
 * log by setting the severity to LogInvalid.
 */
LogMessage::LogMessage(LogMessage &&other)
	: msgStream_(std::move(other.msgStream_)), category_(other.category_),
	  severity_(other.severity_)
{
	other.severity_ = LogInvalid;
}

void LogMessage::init(const char *fileName, unsigned int line)
{
	/* Log the timestamp, severity and file information. */
	timestamp_ = utils::clock::now();

	std::ostringstream ossFileInfo;
	ossFileInfo << utils::basename(fileName) << ":" << line;
	fileInfo_ = ossFileInfo.str();
}

LogMessage::~LogMessage()
{
	/* Don't print anything if we have been moved to another LogMessage. */
	if (severity_ == LogInvalid)
		return;

	Logger *logger = Logger::instance();
	if (!logger)
		return;

	msgStream_ << std::endl;

	if (severity_ >= category_.severity())
		logger->write(*this);

	if (severity_ == LogSeverity::LogFatal) {
		logger->backtrace();
		std::abort();
	}
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
 * \fn LogMessage::timestamp()
 * \brief Retrieve the timestamp of the log message
 * \return The timestamp of the message
 */

/**
 * \fn LogMessage::severity()
 * \brief Retrieve the severity of the log message
 * \return The severity of the message
 */

/**
 * \fn LogMessage::category()
 * \brief Retrieve the category of the log message
 * \return The category of the message
 */

/**
 * \fn LogMessage::fileInfo()
 * \brief Retrieve the file info of the log message
 * \return The file info of the message
 */

/**
 * \fn LogMessage::prefix()
 * \brief Retrieve the prefix of the log message
 * \return The prefix of the message
 */

/**
 * \fn LogMessage::msg()
 * \brief Retrieve the message text of the log message
 * \return The message text of the message, as a string
 */

/**
 * \class Loggable
 * \brief Base class to support log message extensions
 *
 * The Loggable class allows classes to extend log messages without any change
 * to the way the LOG() macro is invoked. By inheriting from Loggable and
 * implementing the logPrefix() virtual function, a class can specify extra
 * information to be automatically added to messages logged from class member
 * function.
 */

Loggable::~Loggable()
{
}

/**
 * \fn Loggable::logPrefix()
 * \brief Retrieve a string to be prefixed to the log message
 *
 * This function allows classes inheriting from the Loggable class to extend the
 * logger with an object-specific prefix output right before the log message
 * contents.
 *
 * \return A string to be prefixed to the log message
 */

/**
 * \brief Create a temporary LogMessage object to log a message
 * \param[in] category The log message category
 * \param[in] severity The log message severity
 * \param[in] fileName The file name where the message is logged from
 * \param[in] line The line number where the message is logged from
 *
 * This function is used as a backend by the LOG() macro to create a log message
 * for locations inheriting from the Loggable class.
 *
 * \return A log message
 */
LogMessage Loggable::_log(const LogCategory *category, LogSeverity severity,
			  const char *fileName, unsigned int line) const
{
	return LogMessage(fileName, line,
			  category ? *category : LogCategory::defaultCategory(),
			  severity, logPrefix());
}

/**
 * \brief Create a temporary LogMessage object to log a message
 * \param[in] category The log message category
 * \param[in] severity The log message severity
 * \param[in] fileName The file name where the message is logged from
 * \param[in] line The line number where the message is logged from
 *
 * This function is used as a backend by the LOG() macro to create a log
 * message for locations not inheriting from the Loggable class.
 *
 * \return A log message
 */
LogMessage _log(const LogCategory *category, LogSeverity severity,
		const char *fileName, unsigned int line)
{
	return LogMessage(fileName, line,
			  category ? *category : LogCategory::defaultCategory(),
			  severity);
}

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
 *
 * \warning Logging from the destructor of a global object, either directly or
 * indirectly, results in undefined behaviour.
 *
 * \todo Allow logging from destructors of global objects to the largest
 * possible extent
 */

/**
 * \def ASSERT(condition)
 * \hideinitializer
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
