/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * utils.cpp - Miscellaneous utility functions
 */

#include <string.h>
#include <sys/auxv.h>

#include "utils.h"

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
 * \returns A pointer to the value in the environment or NULL if the requested
 * environment variable doesn't exist or if secure execution is required.
 */
char *secure_getenv(const char *name)
{
	if (getauxval(AT_SECURE))
		return NULL;

	return getenv(name);
}

/**
 * \fn libcamera::utils::make_unique(Args &&... args)
 * \brief Constructs an object of type T and wraps it in a std::unique_ptr.
 */

} /* namespace utils */

} /* namespace libcamera */
