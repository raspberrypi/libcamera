/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Event dispatcher test
 */

#include <chrono>
#include <iostream>
#include <signal.h>
#include <sys/time.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

static EventDispatcher *dispatcher;
static bool interrupt;

class EventDispatcherTest : public Test
{
protected:
	static void sigAlarmHandler(int)
	{
		cout << "SIGALARM received" << endl;
		if (interrupt)
			dispatcher->interrupt();
	}

	int init()
	{
		dispatcher = Thread::current()->eventDispatcher();

		struct sigaction sa = {};
		sa.sa_handler = &sigAlarmHandler;
		sigaction(SIGALRM, &sa, nullptr);

		return 0;
	}

	int run()
	{
		Timer timer;

		/* Event processing interruption by signal. */
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

		timer.start(1000ms);

		struct itimerval itimer = {};
		itimer.it_value.tv_usec = 500000;
		interrupt = false;
		setitimer(ITIMER_REAL, &itimer, nullptr);

		dispatcher->processEvents();

		std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
		std::chrono::steady_clock::duration duration = stop - start;
		int msecs = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

		if (abs(msecs - 1000) > 50) {
			cout << "Event processing restart test failed" << endl;
			return TestFail;
		}

		/* Event processing interruption. */
		timer.start(1000ms);
		dispatcher->interrupt();

		dispatcher->processEvents();

		if (!timer.isRunning()) {
			cout << "Event processing immediate interruption failed" << endl;
			return TestFail;
		}

		timer.start(1000ms);
		itimer.it_value.tv_usec = 500000;
		interrupt = true;
		setitimer(ITIMER_REAL, &itimer, nullptr);

		dispatcher->processEvents();

		if (!timer.isRunning()) {
			cout << "Event processing delayed interruption failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
	}
};

TEST_REGISTER(EventDispatcherTest)
