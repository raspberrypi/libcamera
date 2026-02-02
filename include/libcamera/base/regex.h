/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * std::regex wrapper for gcc
 */

#include <libcamera/base/private.h>

#pragma GCC diagnostic push
#if defined __SANITIZE_ADDRESS__ && defined __OPTIMIZE__
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <regex>
#pragma GCC diagnostic pop
