/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event-dispatcher.cpp - Event dispatcher test
 */

#include <iostream>
#include <signal.h>
#include <sys/time.h>

#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>
#include <libcamera/timer.h>

#include "test.h"

using namespace std;
using namespace libcamera;

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
		dispatcher = CameraManager::instance()->eventDispatcher();

		struct sigaction sa = {};
		sa.sa_handler = &sigAlarmHandler;
		sigaction(SIGALRM, &sa, nullptr);

		return 0;
	}

	int run()
	{
		Timer timer;

		/* Event processing interruption by signal. */
		struct timespec start;
		clock_gettime(CLOCK_MONOTONIC, &start);

		timer.start(1000);

		struct itimerval itimer = {};
		itimer.it_value.tv_usec = 500000;
		interrupt = false;
		setitimer(ITIMER_REAL, &itimer, nullptr);

		dispatcher->processEvents();

		struct timespec stop;
		clock_gettime(CLOCK_MONOTONIC, &stop);
		int duration = (stop.tv_sec - start.tv_sec) * 1000;
		duration += (stop.tv_nsec - start.tv_nsec) / 1000000;

		if (abs(duration - 1000) > 50) {
			cout << "Event processing restart test failed" << endl;
			return TestFail;
		}

		/* Event processing interruption. */
		timer.start(1000);
		dispatcher->interrupt();

		dispatcher->processEvents();

		if (!timer.isRunning()) {
			cout << "Event processing immediate interruption failed" << endl;
			return TestFail;
		}

		timer.start(1000);
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
