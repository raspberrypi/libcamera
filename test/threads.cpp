/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * threads.cpp - Threads test
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "thread.h"
#include "test.h"

using namespace std;
using namespace libcamera;

class InstrumentedThread : public Thread
{
public:
	InstrumentedThread(unsigned int iterations)
		: iterations_(iterations)
	{
	}

protected:
	void run()
	{
		for (unsigned int i = 0; i < iterations_; ++i) {
			this_thread::sleep_for(chrono::milliseconds(50));
		}
	}

private:
	unsigned int iterations_;
};

class ThreadTest : public Test
{
protected:
	int init()
	{
		return 0;
	}

	int run()
	{
		/* Test Thread() retrieval for the main thread. */
		Thread *thread = Thread::current();
		if (!thread) {
			cout << "Thread::current() failed to main thread"
			     << endl;
			return TestFail;
		}

		if (!thread->isRunning()) {
			cout << "Main thread is not running" << endl;
			return TestFail;
		}

		/* Test starting the main thread, the test shall not crash. */
		thread->start();

		/* Test the running state of a custom thread. */
		thread = new Thread();
		thread->start();

		if (!thread->isRunning()) {
			cout << "Thread is not running after being started"
			     << endl;
			return TestFail;
		}

		thread->exit(0);
		thread->wait();

		if (thread->isRunning()) {
			cout << "Thread is still running after finishing"
			     << endl;
			return TestFail;
		}

		delete thread;

		return TestPass;
	}

	void cleanup()
	{
	}
};

TEST_REGISTER(ThreadTest)
