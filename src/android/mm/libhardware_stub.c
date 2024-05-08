/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (C) 2023, Ideas on Board
 *
 * Android libhardware stub for test compilation
 */

#include <errno.h>

#include <hardware/hardware.h>

int hw_get_module(const char *id __attribute__((__unused__)),
		  const struct hw_module_t **module)
{
	*module = NULL;
	return -ENOTSUP;
}
