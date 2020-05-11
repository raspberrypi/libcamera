/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer.cpp - Timer test
 */

#include <chrono>
#include <iostream>

#include <libcamera/event_dispatcher.h>
#include <libcamera/timer.h>

#include "libcamera/internal/thread.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class ManagedTimer : public Timer
{
public:
	ManagedTimer()
		: Timer(), count_(0)
	{
		timeout.connect(this, &ManagedTimer::timeoutHandler);
	}

	void start(int msec)
	{
		count_ = 0;
		start_ = std::chrono::steady_clock::now();
		expiration_ = std::chrono::steady_clock::time_point();

		Timer::start(msec);
	}

	void start(std::chrono::steady_clock::time_point deadline)
	{
		count_ = 0;
		start_ = std::chrono::steady_clock::now();
		expiration_ = std::chrono::steady_clock::time_point();

		Timer::start(deadline);
	}

	int jitter()
	{
		std::chrono::steady_clock::duration duration = expiration_ - deadline();
		return abs(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
	}

	bool hasFailed()
	{
		return isRunning() || count_ != 1 || jitter() > 50;
	}

private:
	void timeoutHandler(Timer *timer)
	{
		expiration_ = std::chrono::steady_clock::now();
		count_++;
	}

	unsigned int count_;
	std::chrono::steady_clock::time_point start_;
	std::chrono::steady_clock::time_point expiration_;
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
		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		ManagedTimer timer;
		ManagedTimer timer2;

		/* Timer expiration. */
		timer.start(1000);

		if (!timer.isRunning()) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		dispatcher->processEvents();

		if (timer.hasFailed()) {
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

		if (timer.hasFailed()) {
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

		if (timer.hasFailed()) {
			cout << "Timer restart test failed" << endl;
			return TestFail;
		}

		/* Timer restart before expiration. */
		timer.start(50);
		timer.start(100);
		timer.start(150);

		dispatcher->processEvents();

		if (timer.hasFailed()) {
			cout << "Timer restart before expiration test failed" << endl;
			return TestFail;
		}

		/* Timer with absolute deadline. */
		timer.start(std::chrono::steady_clock::now() + std::chrono::milliseconds(200));

		dispatcher->processEvents();

		if (timer.hasFailed()) {
			cout << "Absolute deadline test failed" << endl;
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

		/*
		 * Test that dynamically allocated timers are stopped when
		 * deleted. This will result in a crash on failure.
		 */
		ManagedTimer *dyntimer = new ManagedTimer();
		dyntimer->start(100);
		delete dyntimer;

		timer.start(200);
		dispatcher->processEvents();

		return TestPass;
	}

	void cleanup()
	{
	}
};

TEST_REGISTER(TimerTest)
