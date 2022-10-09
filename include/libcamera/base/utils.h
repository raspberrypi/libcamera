/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * utils.h - Miscellaneous utility functions
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <iterator>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <string.h>
#include <sys/time.h>
#include <type_traits>
#include <utility>
#include <vector>

#include <libcamera/base/private.h>

#ifndef __DOXYGEN__

/* uClibc and uClibc-ng don't provide O_TMPFILE */
#ifndef O_TMPFILE
#define O_TMPFILE (020000000 | O_DIRECTORY)
#endif

#endif

namespace libcamera {

namespace utils {

const char *basename(const char *path);

char *secure_getenv(const char *name);
std::string dirname(const std::string &path);

template<typename T>
std::vector<typename T::key_type> map_keys(const T &map)
{
	std::vector<typename T::key_type> keys;
	std::transform(map.begin(), map.end(), std::back_inserter(keys),
		       [](const auto &value) { return value.first; });
	return keys;
}

template<class InputIt1, class InputIt2>
unsigned int set_overlap(InputIt1 first1, InputIt1 last1,
			 InputIt2 first2, InputIt2 last2)
{
	unsigned int count = 0;

	while (first1 != last1 && first2 != last2) {
		if (*first1 < *first2) {
			++first1;
		} else {
			if (!(*first2 < *first1))
				count++;
			++first2;
		}
	}

	return count;
}

using clock = std::chrono::steady_clock;
using duration = std::chrono::steady_clock::duration;
using time_point = std::chrono::steady_clock::time_point;

struct timespec duration_to_timespec(const duration &value);
std::string time_point_to_string(const time_point &time);

#ifndef __DOXYGEN__
struct _hex {
	uint64_t v;
	unsigned int w;
};

std::basic_ostream<char, std::char_traits<char>> &
operator<<(std::basic_ostream<char, std::char_traits<char>> &stream, const _hex &h);
#endif

template<typename T,
	 std::enable_if_t<std::is_integral<T>::value> * = nullptr>
_hex hex(T value, unsigned int width = 0);

#ifndef __DOXYGEN__
template<>
inline _hex hex<int32_t>(int32_t value, unsigned int width)
{
	return { static_cast<uint64_t>(value), width ? width : 8 };
}

template<>
inline _hex hex<uint32_t>(uint32_t value, unsigned int width)
{
	return { static_cast<uint64_t>(value), width ? width : 8 };
}

template<>
inline _hex hex<int64_t>(int64_t value, unsigned int width)
{
	return { static_cast<uint64_t>(value), width ? width : 16 };
}

template<>
inline _hex hex<uint64_t>(uint64_t value, unsigned int width)
{
	return { static_cast<uint64_t>(value), width ? width : 16 };
}
#endif

size_t strlcpy(char *dst, const char *src, size_t size);

#ifndef __DOXYGEN__
template<typename Container, typename UnaryOp>
std::string join(const Container &items, const std::string &sep, UnaryOp op)
{
	std::ostringstream ss;
	bool first = true;

	for (typename Container::const_iterator it = std::begin(items);
	     it != std::end(items); ++it) {
		if (!first)
			ss << sep;
		else
			first = false;

		ss << op(*it);
	}

	return ss.str();
}

template<typename Container>
std::string join(const Container &items, const std::string &sep)
{
	std::ostringstream ss;
	bool first = true;

	for (typename Container::const_iterator it = std::begin(items);
	     it != std::end(items); ++it) {
		if (!first)
			ss << sep;
		else
			first = false;

		ss << *it;
	}

	return ss.str();
}
#else
template<typename Container, typename UnaryOp>
std::string join(const Container &items, const std::string &sep, UnaryOp op = nullptr);
#endif

namespace details {

class StringSplitter
{
public:
	StringSplitter(const std::string &str, const std::string &delim);

	class iterator
	{
	public:
		using difference_type = std::size_t;
		using value_type = std::string;
		using pointer = value_type *;
		using reference = value_type &;
		using iterator_category = std::input_iterator_tag;

		iterator(const StringSplitter *ss, std::string::size_type pos);

		iterator &operator++();
		std::string operator*() const;
		bool operator!=(const iterator &other) const;

	private:
		const StringSplitter *ss_;
		std::string::size_type pos_;
		std::string::size_type next_;
	};

	iterator begin() const;
	iterator end() const;

private:
	std::string str_;
	std::string delim_;
};

} /* namespace details */

details::StringSplitter split(const std::string &str, const std::string &delim);

std::string toAscii(const std::string &str);

std::string libcameraBuildPath();
std::string libcameraSourcePath();

constexpr unsigned int alignDown(unsigned int value, unsigned int alignment)
{
	return value / alignment * alignment;
}

constexpr unsigned int alignUp(unsigned int value, unsigned int alignment)
{
	return (value + alignment - 1) / alignment * alignment;
}

namespace details {

template<typename T>
struct reverse_adapter {
	T &iterable;
};

template<typename T>
auto begin(reverse_adapter<T> r)
{
	return std::rbegin(r.iterable);
}

template<typename T>
auto end(reverse_adapter<T> r)
{
	return std::rend(r.iterable);
}

} /* namespace details */

template<typename T>
details::reverse_adapter<T> reverse(T &&iterable)
{
	return { iterable };
}

namespace details {

template<typename Base>
class enumerate_iterator
{
private:
	using base_reference = typename std::iterator_traits<Base>::reference;

public:
	using difference_type = typename std::iterator_traits<Base>::difference_type;
	using value_type = std::pair<const std::size_t, base_reference>;
	using pointer = value_type *;
	using reference = value_type &;
	using iterator_category = std::input_iterator_tag;

	explicit enumerate_iterator(Base iter)
		: current_(iter), pos_(0)
	{
	}

	enumerate_iterator &operator++()
	{
		++current_;
		++pos_;
		return *this;
	}

	bool operator!=(const enumerate_iterator &other) const
	{
		return current_ != other.current_;
	}

	value_type operator*() const
	{
		return { pos_, *current_ };
	}

private:
	Base current_;
	std::size_t pos_;
};

template<typename Base>
class enumerate_adapter
{
public:
	using iterator = enumerate_iterator<Base>;

	enumerate_adapter(Base begin, Base end)
		: begin_(begin), end_(end)
	{
	}

	iterator begin() const
	{
		return iterator{ begin_ };
	}

	iterator end() const
	{
		return iterator{ end_ };
	}

private:
	const Base begin_;
	const Base end_;
};

} /* namespace details */

template<typename T>
auto enumerate(T &iterable) -> details::enumerate_adapter<decltype(iterable.begin())>
{
	return { std::begin(iterable), std::end(iterable) };
}

#ifndef __DOXYGEN__
template<typename T, size_t N>
auto enumerate(T (&iterable)[N]) -> details::enumerate_adapter<T *>
{
	return { std::begin(iterable), std::end(iterable) };
}
#endif

class Duration : public std::chrono::duration<double, std::nano>
{
	using BaseDuration = std::chrono::duration<double, std::nano>;

public:
	Duration() = default;

	template<typename Rep>
	constexpr explicit Duration(const Rep &r)
		: BaseDuration(r)
	{
	}

	template<typename Rep, typename Period>
	constexpr Duration(const std::chrono::duration<Rep, Period> &d)
		: BaseDuration(d)
	{
	}

	template<typename Period>
	double get() const
	{
		auto const c = std::chrono::duration_cast<std::chrono::duration<double, Period>>(*this);
		return c.count();
	}

	explicit constexpr operator bool() const
	{
		return *this != BaseDuration::zero();
	}
};

template<typename T>
decltype(auto) abs_diff(const T &a, const T &b)
{
	if (a < b)
		return b - a;
	else
		return a - b;
}

} /* namespace utils */

#ifndef __DOXYGEN__
template<class CharT, class Traits>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &os,
					      const utils::Duration &d);
#endif

} /* namespace libcamera */
