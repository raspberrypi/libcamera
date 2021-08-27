/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event-thread.cpp - Threaded event test
 */

#include <chrono>
#include <iostream>
#include <string.h>
#include <unistd.h>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class EventHandler : public Object
{
public:
	EventHandler()
		: notified_(false)
	{
		int ret = pipe(pipefd_);
		if (ret < 0) {
			ret = errno;
			cout << "pipe() failed: " << strerror(ret) << endl;
		}

		notifier_ = new EventNotifier(pipefd_[0], EventNotifier::Read, this);
		notifier_->activated.connect(this, &EventHandler::readReady);
	}

	~EventHandler()
	{
		delete notifier_;

		close(pipefd_[0]);
		close(pipefd_[1]);
	}

	int notify()
	{
		std::string data("H2G2");
		ssize_t ret;

		memset(data_, 0, sizeof(data_));
		size_ = 0;

		ret = write(pipefd_[1], data.data(), data.size());
		if (ret < 0) {
			cout << "Pipe write failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	bool notified() const
	{
		return notified_;
	}

private:
	void readReady()
	{
		size_ = read(notifier_->fd(), data_, sizeof(data_));
		notified_ = true;
	}

	EventNotifier *notifier_;

	int pipefd_[2];

	bool notified_;
	char data_[16];
	ssize_t size_;
};

class EventThreadTest : public Test
{
protected:
	int run()
	{
		Thread thread;
		thread.start();

		/*
		 * Fire the event notifier and then move the notifier to a
		 * different thread. The notifier will not notice the event
		 * immediately as there is no event dispatcher loop running in
		 * the main thread. This tests that a notifier being moved to a
		 * different thread will correctly process already pending
		 * events in the new thread.
		 */
		EventHandler handler;
		handler.notify();
		handler.moveToThread(&thread);

		this_thread::sleep_for(chrono::milliseconds(100));

		/* Must stop thread before destroying the handler. */
		thread.exit(0);
		thread.wait();

		if (!handler.notified()) {
			cout << "Thread event handling test failed" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(EventThreadTest)
