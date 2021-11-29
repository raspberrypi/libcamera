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

void Test::setArgs([[maybe_unused]] int argc, char *argv[])
{
	self_ = argv[0];
}

int Test::execute()
{
	int ret;

	ret = init();
	if (ret)
		return ret;

	ret = run();

	cleanup();

	return ret;
}
