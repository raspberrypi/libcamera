/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2020, Raspberry Pi (Trading) Limited
 *
 * logging.hpp - logging macros
 */
#pragma once

#include <iostream>

#ifndef RPI_LOGGING_ENABLE
#define RPI_LOGGING_ENABLE 0
#endif

#ifndef RPI_WARNING_ENABLE
#define RPI_WARNING_ENABLE 1
#endif

#define RPI_LOG(stuff)                                                         \
	do {                                                                   \
		if (RPI_LOGGING_ENABLE)                                        \
			std::cout << __FUNCTION__ << ": " << stuff << "\n";    \
	} while (0)

#define RPI_WARN(stuff)                                                        \
	do {                                                                   \
		if (RPI_WARNING_ENABLE)                                        \
			std::cout << __FUNCTION__ << " ***WARNING*** "         \
				  << stuff << "\n";                            \
	} while (0)
