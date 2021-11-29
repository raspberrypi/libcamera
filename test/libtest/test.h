/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * test.h - libcamera test base class
 */

#pragma once

#include <sstream>
#include <string>

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

	void setArgs(int argc, char *argv[]);
	int execute();

	const std::string &self() const { return self_; }

protected:
	virtual int init() { return 0; }
	virtual int run() = 0;
	virtual void cleanup() {}

private:
	std::string self_;
};

#define TEST_REGISTER(Klass)						\
int main(int argc, char *argv[])					\
{									\
	Klass klass;							\
	klass.setArgs(argc, argv);					\
	return klass.execute();						\
}
