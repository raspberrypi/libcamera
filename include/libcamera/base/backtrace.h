/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * backtrace.h - Call stack backtraces
 */
#ifndef __LIBCAMERA_BASE_BACKTRACE_H__
#define __LIBCAMERA_BASE_BACKTRACE_H__

#include <string>
#include <vector>

#include <libcamera/base/private.h>

#include <libcamera/base/class.h>

namespace libcamera {

class Backtrace
{
public:
	Backtrace();

	std::string toString(unsigned int skipLevels = 0) const;

private:
	LIBCAMERA_DISABLE_COPY(Backtrace)

	bool backtraceTrace();
	bool unwindTrace();

	std::vector<void *> backtrace_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_BASE_BACKTRACE_H__ */
