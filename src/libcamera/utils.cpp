/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * utils.cpp - Miscellaneous utility functions
 */

#include "utils.h"

#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/**
 * \file utils.h
 * \brief Miscellaneous utility functions
 */

namespace libcamera {

namespace utils {

/**
 * \def ARRAY_SIZE(array)
 * \brief Determine the number of elements in the static array.
 */

/**
 * \brief Strip the directory prefix from the path
 * \param[in] path The path to process
 *
 * basename is implemented differently across different C libraries. This
 * implementation matches the one provided by the GNU libc, and does not
 * modify its input parameter.
 *
 * \return A pointer within the given path without any leading directory
 * components.
 */
const char *basename(const char *path)
{
       const char *base = strrchr(path, '/');
       return base ? base + 1 : path;
}

/**
 * \brief Get an environment variable
 * \param[in] name The name of the variable to return
 *
 * The environment list is searched to find the variable 'name', and the
 * corresponding string is returned.
 *
 * If 'secure execution' is required then this function always returns NULL to
 * avoid vulnerabilities that could occur if set-user-ID or set-group-ID
 * programs accidentally trust the environment.
 *
 * \return A pointer to the value in the environment or NULL if the requested
 * environment variable doesn't exist or if secure execution is required.
 */
char *secure_getenv(const char *name)
{
#if HAVE_SECURE_GETENV
	return ::secure_getenv(name);
#else
	if (issetugid())
		return NULL;

	return getenv(name);
#endif
}

/**
 * \fn libcamera::utils::set_overlap(InputIt1 first1, InputIt1 last1,
 *				     InputIt2 first2, InputIt2 last2)
 * \brief Count the number of elements in the intersection of two ranges
 *
 * Count the number of elements in the intersection of the sorted ranges [\a
 * first1, \a last1) and [\a first1, \a last2). Elements are compared using
 * operator< and the ranges must be sorted with respect to the same.
 *
 * \return The number of elements in the intersection of the two ranges
 */

/**
 * \fn libcamera::utils::clamp(const T& v, const T& lo, const T& hi)
 * \param[in] v The value to clamp
 * \param[in] lo The lower boundary to clamp v to
 * \param[in] hi The higher boundary to clamp v to
 * \return lo if v is less than lo, hi if v is greater than hi, otherwise v
 */

/**
 * \typedef clock
 * \brief The libcamera clock (monotonic)
 */

/**
 * \typedef duration
 * \brief The libcamera duration related to libcamera::utils::clock
 */

/**
 * \typedef time_point
 * \brief The libcamera time point related to libcamera::utils::clock
 */

/**
 * \brief Convert a duration to a timespec
 * \param[in] value The duration
 * \return A timespec expressing the duration
 */
struct timespec duration_to_timespec(const duration &value)
{
	uint64_t nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(value).count();
	struct timespec ts;
	ts.tv_sec = nsecs / 1000000000ULL;
	ts.tv_nsec = nsecs % 1000000000ULL;
	return ts;
}

/**
 * \brief Convert a time point to a string representation
 * \param[in] time The time point
 * \return A string representing the time point in hh:mm:ss.nanoseconds format
 */
std::string time_point_to_string(const time_point &time)
{
	uint64_t nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
	unsigned int secs = nsecs / 1000000000ULL;

	std::ostringstream ossTimestamp;
	ossTimestamp.fill('0');
	ossTimestamp << secs / (60 * 60) << ":"
		     << std::setw(2) << (secs / 60) % 60 << ":"
		     << std::setw(2) << secs % 60 << "."
		     << std::setw(9) << nsecs % 1000000000ULL;
	return ossTimestamp.str();
}

std::basic_ostream<char, std::char_traits<char>> &
operator<<(std::basic_ostream<char, std::char_traits<char>> &stream, const _hex &h)
{
	stream << "0x";

	std::ostream::fmtflags flags = stream.setf(std::ios_base::hex,
						   std::ios_base::basefield);
	std::streamsize width = stream.width(h.w);
	char fill = stream.fill('0');

	stream << h.v;

	stream.flags(flags);
	stream.width(width);
	stream.fill(fill);

	return stream;
}

/**
 * \fn hex(T value, unsigned int width)
 * \brief Write an hexadecimal value to an output string
 * \param value The value
 * \param width The width
 *
 * Return an object of unspecified type such that, if \a os is the name of an
 * output stream of type std::ostream, and T is an integer type, then the
 * expression
 *
 * \code{.cpp}
 * os << utils::hex(value)
 * \endcode
 *
 * will output the \a value to the stream in hexadecimal form with the base
 * prefix and the filling character set to '0'. The field width is set to \a
 * width if specified to a non-zero value, or to the native width of type T
 * otherwise. The \a os stream configuration is not modified.
 */

/**
 * \brief Copy a string with a size limit
 * \param[in] dst The destination string
 * \param[in] src The source string
 * \param[in] size The size of the destination string
 *
 * This function copies the null-terminated string \a src to \a dst with a limit
 * of \a size - 1 characters, and null-terminates the result if \a size is
 * larger than 0. If \a src is larger than \a size - 1, \a dst is truncated.
 *
 * \return The size of \a src
 */
size_t strlcpy(char *dst, const char *src, size_t size)
{
	if (size) {
		strncpy(dst, src, size);
		dst[size - 1] = '\0';
	}

	return strlen(src);
}

details::StringSplitter::StringSplitter(const std::string &str, const std::string &delim)
	: str_(str), delim_(delim)
{
}

details::StringSplitter::iterator::iterator(const details::StringSplitter *ss, std::string::size_type pos)
	: ss_(ss), pos_(pos)
{
	next_ = ss_->str_.find(ss_->delim_, pos_);
}

details::StringSplitter::iterator &details::StringSplitter::iterator::operator++()
{
	pos_ = next_;
	if (pos_ != std::string::npos) {
		pos_ += ss_->delim_.length();
		next_ = ss_->str_.find(ss_->delim_, pos_);
	}

	return *this;
}

std::string details::StringSplitter::iterator::operator*() const
{
	std::string::size_type count;
	count = next_ != std::string::npos ? next_ - pos_ : next_;
	return ss_->str_.substr(pos_, count);
}

bool details::StringSplitter::iterator::operator!=(const details::StringSplitter::iterator &other) const
{
	return pos_ != other.pos_;
}

details::StringSplitter::iterator details::StringSplitter::begin() const
{
	return iterator(this, 0);
}

details::StringSplitter::iterator details::StringSplitter::end() const
{
	return iterator(this, std::string::npos);
}

/**
 * \fn split(const std::string &str, const std::string &delim)
 * \brief Split a string based on a delimiter
 * \param[in] str The string to split
 * \param[in] delim The delimiter string
 *
 * This function splits the string \a str into substrings based on the
 * delimiter \a delim. It returns an object of unspecified type that can be
 * used in a range-based for loop and yields the substrings in sequence.
 *
 * \return An object that can be used in a range-based for loop to iterate over
 * the substrings
 */
details::StringSplitter split(const std::string &str, const std::string &delim)
{
	/** \todo Try to avoid copies of str and delim */
	return details::StringSplitter(str, delim);
}

} /* namespace utils */

} /* namespace libcamera */
