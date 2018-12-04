/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * main.cpp - libcamera main class
 */

#include <iostream>
#include <libcamera/libcamera.h>

using std::cout;
using std::endl;

void libcamera::init_lib(void)
{
	cout << "Lib Camera Init" << endl;
}
