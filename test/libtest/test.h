/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * test.h - libcamera test base class
 */

#pragma once

#include <sstream>

enum TestStatus {
	TestPass = 0,
	TestFail = -1,
	TestSkip = 77,
};

class Test
{
public:
	Test();
	virtual ~Test();

	int execute();

protected:
	virtual int init() { return 0; }
	virtual int run() = 0;
	virtual void cleanup() {}
};

#define TEST_REGISTER(klass)						\
int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[])	\
{									\
	return klass().execute();					\
}
