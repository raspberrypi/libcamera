/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * Miscellaneous utility functions
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <ostream>
#include <sstream>
#include <stdint.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <type_traits>
#include <utility>
#include <vector>

#include <libcamera/base/class.h>
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

namespace details {

struct hex {
	uint64_t v;
	unsigned int w;
};

template<typename T>
constexpr unsigned int hex_width()
{
	return sizeof(T) * 2;
}

std::basic_ostream<char, std::char_traits<char>> &
operator<<(std::basic_ostream<char, std::char_traits<char>> &stream, const hex &h);

} /* namespace details */

template<typename T, std::enable_if_t<std::is_integral_v<T>> * = nullptr>
details::hex hex(T value, unsigned int width = details::hex_width<T>())
{
	return { static_cast<std::make_unsigned_t<T>>(value), width };
}

size_t strlcpy(char *dst, const char *src, size_t size);

#ifndef __DOXYGEN__
template<typename Container, typename UnaryOp>
std::string join(const Container &items, const std::string &sep, UnaryOp op)
{
	std::ostringstream ss;
	bool first = true;

	for (auto it = std::begin(items); it != std::end(items); ++it) {
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

	for (auto it = std::begin(items); it != std::end(items); ++it) {
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

		bool operator==(const iterator &other) const
		{
			return pos_ == other.pos_;
		}

		bool operator!=(const iterator &other) const
		{
			return !(*this == other);
		}

	private:
		const StringSplitter *ss_;
		std::string::size_type pos_;
		std::string::size_type next_;
	};

	iterator begin() const
	{
		return { this, 0 };
	}

	iterator end() const
	{
		return { this, std::string::npos };
	}

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
auto enumerate(T &iterable)
{
	return details::enumerate_adapter{ std::begin(iterable), std::end(iterable) };
}

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

	constexpr Duration operator-() const
	{
		return BaseDuration::operator-();
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

double strtod(const char *__restrict nptr, char **__restrict endptr);

template<class Enum>
constexpr std::underlying_type_t<Enum> to_underlying(Enum e) noexcept
{
	return static_cast<std::underlying_type_t<Enum>>(e);
}

class ScopeExitActions
{
public:
	~ScopeExitActions();

	void operator+=(std::function<void()> &&action);
	void release();

private:
	std::vector<std::function<void()>> actions_;
};

#ifndef __DOXYGEN__
template<typename EF>
class scope_exit
{
public:
	template<typename Fn,
		 std::enable_if_t<!std::is_same_v<std::remove_cv_t<std::remove_reference_t<Fn>>, scope_exit> &&
				  std::is_constructible_v<EF, Fn>> * = nullptr>
	explicit scope_exit(Fn &&fn)
		: exitFunction_(std::forward<Fn>(fn))
	{
		static_assert(std::is_nothrow_constructible_v<EF, Fn>);
	}

	~scope_exit()
	{
		if (active_)
			exitFunction_();
	}

	void release()
	{
		active_ = false;
	}

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(scope_exit)

	EF exitFunction_;
	bool active_ = true;
};

template<typename EF>
scope_exit(EF) -> scope_exit<EF>;

#endif /* __DOXYGEN__ */

} /* namespace utils */

#ifndef __DOXYGEN__
std::ostream &operator<<(std::ostream &os, const utils::Duration &d);
#endif

} /* namespace libcamera */
