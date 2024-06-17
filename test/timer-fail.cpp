/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Threaded timer failure test
 */

#include <chrono>
#include <iostream>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/object.h>
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
	}

	void start()
	{
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

class TimerFailTest : public Test
{
protected:
	int init()
	{
		thread_.start();

		timeout_ = new TimeoutHandler();
		timeout_->moveToThread(&thread_);

		return TestPass;
	}

	int run()
	{
		/*
		 * Test that the forbidden operation of starting the timer from
		 * another thread results in a failure. We need to interrupt the
		 * event dispatcher to make sure we don't succeed simply because
		 * the event dispatcher hasn't noticed the timer restart.
		 */
		timeout_->start();
		thread_.eventDispatcher()->interrupt();

		this_thread::sleep_for(chrono::milliseconds(200));

		/*
		 * The wrong start() call should result in an assertion in debug
		 * builds, and a timeout in release builds. The test is
		 * therefore marked in meson.build as expected to fail. We need
		 * to return TestPass in the unexpected (usually known as
		 * "fail") case, and TestFail otherwise.
		 */
		if (timeout_->timeout()) {
			cout << "Timer start from wrong thread succeeded unexpectedly"
			     << endl;
			return TestPass;
		}

		return TestFail;
	}

	void cleanup()
	{
		/*
		 * Object class instances must be destroyed from the thread
		 * they live in.
		 */
		timeout_->deleteLater();
		thread_.exit(0);
		thread_.wait();
	}

private:
	TimeoutHandler *timeout_;
	Thread thread_;
};

TEST_REGISTER(TimerFailTest)
