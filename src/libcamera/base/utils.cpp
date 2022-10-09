/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * utils.cpp - Miscellaneous utility functions
 */

#include <libcamera/base/utils.h>

#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/**
 * \file base/utils.h
 * \brief Miscellaneous utility functions
 */

namespace libcamera {

namespace utils {

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
 * \note Not all platforms may support the features required to implement the
 * secure execution check, in which case this function behaves as getenv(). A
 * notable example of this is Android.
 *
 * \return A pointer to the value in the environment or NULL if the requested
 * environment variable doesn't exist or if secure execution is required.
 */
char *secure_getenv(const char *name)
{
#if HAVE_SECURE_GETENV
	return ::secure_getenv(name);
#else
#if HAVE_ISSETUGID
	if (issetugid())
		return NULL;
#endif
	return getenv(name);
#endif
}

/**
 * \brief Identify the dirname portion of a path
 * \param[in] path The full path to parse
 *
 * This function conforms with the behaviour of the %dirname() function as
 * defined by POSIX.
 *
 * \return A string of the directory component of the path
 */
std::string dirname(const std::string &path)
{
	if (path.empty())
		return ".";

	/*
	 * Skip all trailing slashes. If the path is only made of slashes,
	 * return "/".
	 */
	size_t pos = path.size() - 1;
	while (path[pos] == '/') {
		if (!pos)
			return "/";
		pos--;
	}

	/*
	 * Find the previous slash. If the path contains no non-trailing slash,
	 * return ".".
	 */
	while (path[pos] != '/') {
		if (!pos)
			return ".";
		pos--;
	}

	/*
	 * Return the directory name up to (but not including) any trailing
	 * slash. If this would result in an empty string, return "/".
	 */
	while (path[pos] == '/') {
		if (!pos)
			return "/";
		pos--;
	}

	return path.substr(0, pos + 1);
}

/**
 * \fn std::vector<typename T::key_type> map_keys(const T &map)
 * \brief Retrieve the keys of a std::map<>
 * \param[in] map The map whose keys to retrieve
 * \return A std::vector<> containing the keys of \a map
 */

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
 * \fn template<typename Container, typename UnaryOp> \
 * std::string utils::join(const Container &items, const std::string &sep, UnaryOp op)
 * \brief Join elements of a container in a string with a separator
 * \param[in] items The container
 * \param[in] sep The separator to add between elements
 * \param[in] op A function that converts individual elements to strings
 *
 * This function joins all elements in the \a items container into a string and
 * returns it. The \a sep separator is added between elements. If the container
 * elements are not implicitly convertible to std::string, the \a op function
 * shall be provided to perform conversion of elements to std::string.
 *
 * \return A string that concatenates all elements in the container
 */

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

/**
 * \brief Remove any non-ASCII characters from a string
 * \param[in] str The string to strip
 *
 * Remove all non-ASCII characters from a string.
 *
 * \return A string equal to \a str stripped out of all non-ASCII characters
 */
std::string toAscii(const std::string &str)
{
	std::string ret;
	for (const char &c : str)
		if (!(c & 0x80))
			ret += c;
	return ret;
}

/**
 * \fn alignDown(unsigned int value, unsigned int alignment)
 * \brief Align \a value down to \a alignment
 * \param[in] value The value to align
 * \param[in] alignment The alignment
 * \return The value rounded down to the nearest multiple of \a alignment
 */

/**
 * \fn alignUp(unsigned int value, unsigned int alignment)
 * \brief Align \a value up to \a alignment
 * \param[in] value The value to align
 * \param[in] alignment The alignment
 * \return The value rounded up to the nearest multiple of \a alignment
 */

/**
 * \fn reverse(T &&iterable)
 * \brief Wrap an iterable to reverse iteration in a range-based loop
 * \param[in] iterable The iterable
 * \return A value of unspecified type that, when used in a range-based for
 * loop, will cause the loop to iterate over the \a iterable in reverse order
 */

/**
 * \fn enumerate(T &iterable)
 * \brief Wrap an iterable to enumerate index and value in a range-based loop
 * \param[in] iterable The iterable
 *
 * Range-based for loops are handy and widely preferred in C++, but are limited
 * in their ability to replace for loops that require access to a loop counter.
 * The enumerate() function solves this problem by wrapping the \a iterable in
 * an adapter that, when used as a range-expression, will provide iterators
 * whose value_type is a pair of index and value reference.
 *
 * The iterable must support std::begin() and std::end(). This includes all
 * containers provided by the standard C++ library, as well as C-style arrays.
 *
 * A typical usage pattern would use structured binding to store the index and
 * value in two separate variables:
 *
 * \code{.cpp}
 * std::vector<int> values = ...;
 *
 * for (auto [index, value] : utils::enumerate(values)) {
 * 	...
 * }
 * \endcode
 *
 * Note that the argument to enumerate() has to be an lvalue, as the lifetime
 * of any rvalue would not be extended to the whole for loop. The compiler will
 * complain if an rvalue is passed to the function, in which case it should be
 * stored in a local variable before the loop.
 *
 * \return A value of unspecified type that, when used in a range-based for
 * loop, iterates over an indexed view of the \a iterable
 */

/**
 * \class Duration
 * \brief Helper class from std::chrono::duration that represents a time
 * duration in nanoseconds with double precision
 */

/**
 * \fn Duration::Duration(const Rep &r)
 * \brief Construct a Duration with \a r ticks
 * \param[in] r The number of ticks
 *
 * The constructed \a Duration object is internally represented in double
 * precision with \a r nanoseconds ticks.
 */

/**
 * \fn Duration::Duration(const std::chrono::duration<Rep, Period> &d)
 * \brief Construct a Duration by converting an arbitrary std::chrono::duration
 * \param[in] d The std::chrono::duration object to convert from
 *
 * The constructed \a Duration object is internally represented in double
 * precision with nanoseconds ticks.
 */

/**
 * \fn Duration::get<Period>()
 * \brief Retrieve the tick count, converted to the timebase provided by the
 * template argument Period of type \a std::ratio
 *
 * A typical usage example is given below:
 *
 * \code{.cpp}
 * utils::Duration d = 5s;
 * double d_in_ms = d.get<std::milli>();
 * \endcode
 *
 * \return The tick count of the Duration expressed in \a Period
 */

/**
 * \fn Duration::operator bool()
 * \brief Boolean operator to test if a \a Duration holds a non-zero time value
 *
 * \return True if \a Duration is a non-zero time value, False otherwise
 */

/**
 * \fn abs_diff(const T& a, const T& b)
 * \brief Calculates the absolute value of the difference between two elements
 * \param[in] a The first element
 * \param[in] b The second element
 *
 * This function calculates the absolute value of the difference between two
 * elements of the same type, in such a way that a negative value will never
 * occur during the calculation.
 *
 * This is inspired by the std::abs_diff() candidate proposed in N4318
 * (http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2014/n4318.pdf).
 *
 * \return The absolute value of the difference of the two parameters \a a and
 * \a b
 */

} /* namespace utils */

#ifndef __DOXYGEN__
template<class CharT, class Traits>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &os,
					      const utils::Duration &d)
{
	std::basic_ostringstream<CharT, Traits> s;

	s.flags(os.flags());
	s.imbue(os.getloc());
	s.setf(std::ios_base::fixed, std::ios_base::floatfield);
	s.precision(2);
	s << d.get<std::micro>() << "us";
	return os << s.str();
}

template
std::basic_ostream<char, std::char_traits<char>> &
operator<< <char, std::char_traits<char>>(std::basic_ostream<char, std::char_traits<char>> &os,
					  const utils::Duration &d);
#endif

} /* namespace libcamera */
