/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * object-invoke.cpp - Cross-thread Object method invocation test
 */

#include <iostream>
#include <thread>

#include <libcamera/event_dispatcher.h>
#include <libcamera/object.h>

#include "test.h"
#include "thread.h"

using namespace std;
using namespace libcamera;

class InvokedObject : public Object
{
public:
	enum Status {
		NoCall,
		InvalidThread,
		CallReceived,
	};

	InvokedObject()
		: status_(NoCall)
	{
	}

	Status status() const { return status_; }
	int value() const { return value_; }
	void reset()
	{
		status_ = NoCall;
		value_ = 0;
	}

	void method(int value)
	{
		if (Thread::current() != thread())
			status_ = InvalidThread;
		else
			status_ = CallReceived;

		value_ = value;
	}

private:
	Status status_;
	int value_;
};

class ObjectInvokeTest : public Test
{
protected:
	int run()
	{
		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		InvokedObject object;

		/*
		 * Test that queued method invocation in the same thread goes
		 * through the event dispatcher.
		 */
		object.invokeMethod(&InvokedObject::method,
				    ConnectionTypeQueued, 42);

		if (object.status() != InvokedObject::NoCall) {
			cerr << "Method not invoked asynchronously" << endl;
			return TestFail;
		}

		dispatcher->processEvents();

		switch (object.status()) {
		case InvokedObject::NoCall:
			cout << "Method not invoked for main thread" << endl;
			return TestFail;
		case InvokedObject::InvalidThread:
			cout << "Method invoked in incorrect thread for main thread" << endl;
			return TestFail;
		default:
			break;
		}

		if (object.value() != 42) {
			cout << "Method invoked with incorrect value for main thread" << endl;
			return TestFail;
		}

		/*
		 * Move the object to a thread and verify that auto method
		 * invocation is delivered in the correct thread.
		 */
		object.reset();
		object.moveToThread(&thread_);

		thread_.start();

		object.invokeMethod(&InvokedObject::method,
				    ConnectionTypeBlocking, 42);

		switch (object.status()) {
		case InvokedObject::NoCall:
			cout << "Method not invoked for custom thread" << endl;
			return TestFail;
		case InvokedObject::InvalidThread:
			cout << "Method invoked in incorrect thread for custom thread" << endl;
			return TestFail;
		default:
			break;
		}

		if (object.value() != 42) {
			cout << "Method invoked with incorrect value for custom thread" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		thread_.exit(0);
		thread_.wait();
	}

private:
	Thread thread_;
};

TEST_REGISTER(ObjectInvokeTest)
