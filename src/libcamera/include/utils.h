/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * utils.h - Miscellaneous utility functions
 */
#ifndef __LIBCAMERA_UTILS_H__
#define __LIBCAMERA_UTILS_H__

#include <memory>

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof(a[0]))

namespace libcamera {

namespace utils {

const char *basename(const char *path);

/* C++11 doesn't provide std::make_unique */
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

char *secure_getenv(const char *name);

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

} /* namespace utils */

} /* namespace libcamera */

#endif /* __LIBCAMERA_UTILS_H__ */
