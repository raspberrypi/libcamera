/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * init.cpp - libcamera initialization test
 */

#include <libcamera/libcamera.h>

int main(void)
{
	libcamera l = libcamera();
	l.init_lib();

	return 0;
}
