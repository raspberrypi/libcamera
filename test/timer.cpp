/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer.cpp - Timer test
 */

#include <iostream>

#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>
#include <libcamera/timer.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class ManagedTimer : public Timer
{
public:
	ManagedTimer()
		: Timer(), interval_(0)
	{
		timeout.connect(this, &ManagedTimer::timeoutHandler);
	}

	void start(int msec)
	{
		interval_ = msec;
		clock_gettime(CLOCK_MONOTONIC, &start_);
		expiration_ = { 0, 0 };

		Timer::start(msec);
	}

	int jitter()
	{
		int duration = (expiration_.tv_sec - start_.tv_sec) * 1000;
		duration += (expiration_.tv_nsec - start_.tv_nsec) / 1000000;
		return abs(duration - interval_);
	}

private:
	void timeoutHandler(Timer *timer)
	{
		clock_gettime(CLOCK_MONOTONIC, &expiration_);
	}

	int interval_;
	struct timespec start_;
	struct timespec expiration_;
};

class TimerTest : public Test
{
protected:
	int init()
	{
		return 0;
	}

	int run()
	{
		EventDispatcher *dispatcher = CameraManager::instance()->eventDispatcher();
		ManagedTimer timer;
		ManagedTimer timer2;

		/* Timer expiration. */
		timer.start(1000);

		if (!timer.isRunning()) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		dispatcher->processEvents();

		if (timer.isRunning() || timer.jitter() > 50) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		/*
		 * 32 bit wrap test
		 * Nanosecond resolution in a 32 bit value wraps at 4.294967
		 * seconds (0xFFFFFFFF / 1000000)
		 */
		timer.start(4295);
		dispatcher->processEvents();

		if (timer.isRunning() || timer.jitter() > 50) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		/* Timer restart. */
		timer.start(500);

		if (!timer.isRunning()) {
			cout << "Timer restart test failed" << endl;
			return TestFail;
		}

		dispatcher->processEvents();

		if (timer.isRunning() || timer.jitter() > 50) {
			cout << "Timer restart test failed" << endl;
			return TestFail;
		}

		/* Two timers. */
		timer.start(1000);
		timer2.start(300);

		dispatcher->processEvents();

		if (!timer.isRunning()) {
			cout << "Two timers test failed" << endl;
			return TestFail;
		}

		if (timer2.jitter() > 50) {
			cout << "Two timers test failed" << endl;
			return TestFail;
		}

		dispatcher->processEvents();

		if (timer.jitter() > 50) {
			cout << "Two timers test failed" << endl;
			return TestFail;
		}

		/* Restart timer before expiration. */
		timer.start(1000);
		timer2.start(300);

		dispatcher->processEvents();

		if (timer2.jitter() > 50) {
			cout << "Two timers test failed" << endl;
			return TestFail;
		}

		timer.start(1000);

		dispatcher->processEvents();

		if (timer.jitter() > 50) {
			cout << "Two timers test failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
	}
};

TEST_REGISTER(TimerTest)
