/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer-thread.cpp - Threaded timer test
 */

#include <chrono>
#include <iostream>

#include <libcamera/timer.h>

#include "test.h"
#include "thread.h"

using namespace std;
using namespace libcamera;

class TimeoutHandler : public Object
{
public:
	TimeoutHandler()
		: timeout_(false)
	{
		timer_.timeout.connect(this, &TimeoutHandler::timeoutHandler);
		timer_.start(100);
	}

	bool timeout() const
	{
		return timeout_;
	}

	void moveToThread(Thread *thread)
	{
		Object::moveToThread(thread);
		timer_.moveToThread(thread);
	}

private:
	void timeoutHandler(Timer *timer)
	{
		timeout_ = true;
	}

	Timer timer_;
	bool timeout_;
};

class TimerThreadTest : public Test
{
protected:
	int run()
	{
		Thread thread;
		thread.start();

		TimeoutHandler timeout;
		timeout.moveToThread(&thread);

		this_thread::sleep_for(chrono::milliseconds(100));

		/* Must stop thread before destroying timeout. */
		thread.exit(0);
		thread.wait();

		if (!timeout.timeout()) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

private:
};

TEST_REGISTER(TimerThreadTest)
