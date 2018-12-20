/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * test.h - libcamera test base class
 */
#ifndef __TEST_TEST_H__
#define __TEST_TEST_H__

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
	virtual void cleanup() { }
};

#define TEST_REGISTER(klass)						\
int main(int argc, char *argv[])					\
{									\
	return klass().execute();					\
}

#endif /* __TEST_TEST_H__ */
