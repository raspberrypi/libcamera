/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * main.cpp - libcamera main class
 */

#include <libcamera/libcamera.h>

#include "log.h"

namespace libcamera {

void libcamera::init_lib(void)
{
	LOG(Info) << "Lib Camera Init";
}

};
