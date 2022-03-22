/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer-thread.cpp - Threaded timer test
 */

#include <chrono>
#include <iostream>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

class TimeoutHandler : public Object
{
public:
	TimeoutHandler()
		: timer_(this), timeout_(false)
	{
		timer_.timeout.connect(this, &TimeoutHandler::timeoutHandler);
		timer_.start(100ms);
	}

	void restart()
	{
		timeout_ = false;
		timer_.start(100ms);
	}

	bool timeout() const
	{
		return timeout_;
	}

private:
	void timeoutHandler()
	{
		timeout_ = true;
	}

	Timer timer_;
	bool timeout_;
};

class TimerThreadTest : public Test
{
protected:
	int init()
	{
		thread_.start();
		timeout_.moveToThread(&thread_);

		return TestPass;
	}

	int run()
	{
		/*
		 * Test that the timer expires and emits the timeout signal in
		 * the thread it belongs to.
		 */
		this_thread::sleep_for(chrono::milliseconds(200));

		if (!timeout_.timeout()) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		/*
		 * Test that starting the timer from another thread fails. We
		 * need to interrupt the event dispatcher to make sure we don't
		 * succeed simply because the event dispatcher hasn't noticed
		 * the timer restart.
		 */
		timeout_.restart();
		thread_.eventDispatcher()->interrupt();

		this_thread::sleep_for(chrono::milliseconds(200));

		if (timeout_.timeout()) {
			cout << "Timer restart test failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		/* Must stop thread before destroying timeout. */
		thread_.exit(0);
		thread_.wait();
	}

private:
	TimeoutHandler timeout_;
	Thread thread_;
};

TEST_REGISTER(TimerThreadTest)
