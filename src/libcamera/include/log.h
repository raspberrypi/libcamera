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
	LogInvalid = -1,
	LogDebug = 0,
	LogInfo,
	LogWarning,
	LogError,
	LogFatal,
};

class LogCategory
{
public:
	explicit LogCategory(const char *name);
	~LogCategory();

	const char *name() const { return name_; }
	LogSeverity severity() const { return severity_; }
	void setSeverity(LogSeverity severity);

	static const LogCategory &defaultCategory();

private:
	const char *name_;
	LogSeverity severity_;
};

#define LOG_DECLARE_CATEGORY(name)					\
extern const LogCategory &_LOG_CATEGORY(name)();

#define LOG_DEFINE_CATEGORY(name)					\
const LogCategory &_LOG_CATEGORY(name)()				\
{									\
	static LogCategory category(#name);				\
	return category;						\
}

class LogMessage
{
public:
	LogMessage(const char *fileName, unsigned int line,
		   LogSeverity severity);
	LogMessage(const char *fileName, unsigned int line,
		   const LogCategory &category, LogSeverity severity);
	LogMessage(const LogMessage &) = delete;
	LogMessage(LogMessage &&);
	~LogMessage();

	std::ostream &stream() { return msgStream_; }

private:
	void init(const char *fileName, unsigned int line);

	std::ostringstream msgStream_;
	const LogCategory &category_;
	LogSeverity severity_;
};

class Loggable
{
public:
	virtual ~Loggable();

protected:
	virtual std::string logPrefix() const = 0;

	LogMessage _log(const char *file, unsigned int line,
			LogSeverity severity) const;
	LogMessage _log(const char *file, unsigned int line,
			const LogCategory &category,
			LogSeverity severity) const;
};

LogMessage _log(const char *file, unsigned int line, LogSeverity severity);
LogMessage _log(const char *file, unsigned int line,
		const LogCategory &category, LogSeverity severity);

#ifndef __DOXYGEN__
#define _LOG_CATEGORY(name) logCategory##name

#define _LOG1(severity) \
	_log(__FILE__, __LINE__, Log##severity).stream()
#define _LOG2(category, severity) \
	_log(__FILE__, __LINE__, _LOG_CATEGORY(category)(), Log##severity).stream()

/*
 * Expand the LOG() macro to _LOG1() or _LOG2() based on the number of
 * arguments.
 */
#define _LOG_MACRO(_1, _2, NAME, ...) NAME
#define LOG(...) _LOG_MACRO(__VA_ARGS__, _LOG2, _LOG1)(__VA_ARGS__)
#else /* __DOXYGEN___ */
#define LOG(category, severity)
#endif /* __DOXYGEN__ */

#ifndef NDEBUG
#define ASSERT(condition) static_cast<void>(({				\
	if (!(condition))						\
		LOG(Fatal) << "assertion \"" #condition "\" failed";	\
}))
#else
#define ASSERT(condition) static_cast<void>(false && (condition))
#endif

} /* namespace libcamera */

#endif /* __LIBCAMERA_LOG_H__ */
