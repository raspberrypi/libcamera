/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Cross-thread signal delivery test
 */

#include <chrono>
#include <iostream>
#include <thread>

#include <libcamera/base/message.h>
#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/utils.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class SignalReceiver : public Object
{
public:
	enum Status {
		NoSignal,
		InvalidThread,
		SignalReceived,
	};

	SignalReceiver()
		: status_(NoSignal)
	{
	}

	Status status() const { return status_; }
	int value() const { return value_; }
	void reset()
	{
		status_ = NoSignal;
		value_ = 0;
	}

	void slot(int value)
	{
		if (Thread::current() != thread())
			status_ = InvalidThread;
		else
			status_ = SignalReceived;

		value_ = value;
	}

private:
	Status status_;
	int value_;
};

class SignalThreadsTest : public Test
{
protected:
	int init()
	{
		receiver_ = new SignalReceiver();
		signal_.connect(receiver_, &SignalReceiver::slot);

		return TestPass;
	}

	int run()
	{
		/* Test that a signal is received in the main thread. */
		signal_.emit(0);

		switch (receiver_->status()) {
		case SignalReceiver::NoSignal:
			cout << "No signal received for direct connection" << endl;
			return TestFail;
		case SignalReceiver::InvalidThread:
			cout << "Signal received in incorrect thread "
				"for direct connection" << endl;
			return TestFail;
		default:
			break;
		}

		/*
		 * Move the object to a thread and verify that the signal is
		 * correctly delivered, with the correct data.
		 */
		receiver_->reset();
		receiver_->moveToThread(&thread_);

		thread_.start();

		signal_.emit(42);

		this_thread::sleep_for(chrono::milliseconds(100));

		switch (receiver_->status()) {
		case SignalReceiver::NoSignal:
			cout << "No signal received for message connection" << endl;
			return TestFail;
		case SignalReceiver::InvalidThread:
			cout << "Signal received in incorrect thread "
				"for message connection" << endl;
			return TestFail;
		default:
			break;
		}

		if (receiver_->value() != 42) {
			cout << "Signal received with incorrect value" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		receiver_->deleteLater();
		thread_.exit(0);
		thread_.wait();
	}

private:
	SignalReceiver *receiver_;
	Thread thread_;

	Signal<int> signal_;
};

TEST_REGISTER(SignalThreadsTest)
