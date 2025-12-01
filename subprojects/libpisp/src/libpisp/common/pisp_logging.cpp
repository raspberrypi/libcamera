/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * pisp_logging.cpp - PiSP logging library
 */

#include "logging.hpp"

#if PISP_LOGGING_ENABLE

#include <cstdlib>
#include <mutex>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

namespace logging = boost::log;
namespace sinks = boost::log::sinks;
namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;
namespace trivial = boost::log::trivial;

namespace libpisp
{

namespace {

std::mutex mutex;

boost::shared_ptr<sinks::synchronous_sink<sinks::text_ostream_backend>> console;
boost::shared_ptr<sinks::synchronous_sink<sinks::text_file_backend>> file;

void logging_file_init(const char *filename, unsigned int level)
{
	logging::formatter format =
		expr::format("[libpisp %1%] %2%")
		% expr::attr<trivial::severity_level>("Severity")
		% expr::smessage;

	// file sink
	file = logging::add_file_log(
			keywords::file_name = filename,
			keywords::rotation_size = 1 * 1024 * 1024,
			keywords::open_mode = std::ios_base::trunc,
			keywords::filter = trivial::severity >= level);
	file->set_formatter(format);
	file->locked_backend()->auto_flush(true);
}

} // namespace

void logging_init()
{
	std::scoped_lock<std::mutex> l(mutex);
	// Default to "warning" level.
	unsigned int level = 3;

	// Can only initialise the console logging once.
	if (console)
		return;

	logging::add_common_attributes();

	logging::formatter format =
		expr::format("[libpisp %1%] %2%")
		% expr::attr<trivial::severity_level>("Severity")
		% expr::smessage;

	char *lev = std::getenv("LIBPISP_LOG_LEVEL");
	if (lev)
		level = std::atoi(lev);

	// Console sink.
	console = logging::add_console_log(std::clog);
	console->set_formatter(format);
	console->set_filter(trivial::severity >= level);

	// File sink if needed.
	const char *log_file = std::getenv("LIBPISP_LOG_FILE");
	if (log_file)
	{
		lev = std::getenv("LIBPISP_LOG_FILE_LEVEL");
		if (lev)
			level = std::atoi(lev);

		logging_file_init(log_file, level);
	}
}

} // namespace libpisp

#else

void libpisp::logging_init()
{
}

#endif // PISP_LOGGING_ENABLE
