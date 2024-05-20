/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Timer test
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

class ManagedTimer : public Timer
{
public:
	ManagedTimer()
		: Timer(), count_(0)
	{
		timeout.connect(this, &ManagedTimer::timeoutHandler);
	}

	void start(std::chrono::milliseconds msec)
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
	void timeoutHandler()
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
		timer.start(1000ms);

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
		timer.start(4295ms);
		dispatcher->processEvents();

		if (timer.hasFailed()) {
			cout << "Timer expiration test failed" << endl;
			return TestFail;
		}

		/* Timer restart. */
		timer.start(500ms);

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
		timer.start(50ms);
		timer.start(100ms);
		timer.start(150ms);

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
		timer.start(1000ms);
		timer2.start(300ms);

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
		timer.start(1000ms);
		timer2.start(300ms);

		dispatcher->processEvents();

		if (timer2.jitter() > 50) {
			cout << "Two timers test failed" << endl;
			return TestFail;
		}

		timer.start(1000ms);

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
		dyntimer->start(100ms);
		delete dyntimer;

		timer.start(200ms);
		dispatcher->processEvents();

		return TestPass;
	}

	void cleanup()
	{
	}
};

TEST_REGISTER(TimerTest)
