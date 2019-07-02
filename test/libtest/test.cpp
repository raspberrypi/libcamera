/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * test.cpp - libcamera test base class
 */

#include <stdlib.h>

#include "test.h"

Test::Test()
{
}

Test::~Test()
{
}

int Test::execute()
{
	int ret;

	ret = setenv("LIBCAMERA_IPA_MODULE_PATH", "src/ipa", 1);
	if (ret)
		return errno;

	ret = setenv("LIBCAMERA_IPA_PROXY_PATH", "src/libcamera/proxy/worker", 1);
	if (ret)
		return errno;

	ret = init();
	if (ret)
		return ret;

	ret = run();

	cleanup();

	return ret;
}
