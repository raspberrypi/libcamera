/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Cross-thread Object method invocation test
 */

#include <iostream>
#include <thread>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

#include "test.h"

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

	void methodWithReference([[maybe_unused]] const int &value)
	{
	}

	int methodWithReturn()
	{
		return 42;
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

		/*
		 * Test that queued method invocation in the same thread goes
		 * through the event dispatcher.
		 */
		object_.invokeMethod(&InvokedObject::method,
				     ConnectionTypeQueued, 42);

		if (object_.status() != InvokedObject::NoCall) {
			cerr << "Method not invoked asynchronously" << endl;
			return TestFail;
		}

		dispatcher->processEvents();

		switch (object_.status()) {
		case InvokedObject::NoCall:
			cout << "Method not invoked for main thread" << endl;
			return TestFail;
		case InvokedObject::InvalidThread:
			cout << "Method invoked in incorrect thread for main thread" << endl;
			return TestFail;
		default:
			break;
		}

		if (object_.value() != 42) {
			cout << "Method invoked with incorrect value for main thread" << endl;
			return TestFail;
		}

		/*
		 * Test that blocking invocation is delivered directly when the
		 * caller and callee live in the same thread.
		 */
		object_.reset();

		object_.invokeMethod(&InvokedObject::method,
				     ConnectionTypeBlocking, 42);

		switch (object_.status()) {
		case InvokedObject::NoCall:
			cout << "Method not invoked for main thread (blocking)" << endl;
			return TestFail;
		case InvokedObject::InvalidThread:
			cout << "Method invoked in incorrect thread for main thread (blocking)" << endl;
			return TestFail;
		default:
			break;
		}

		/*
		 * Move the object to a thread and verify that auto method
		 * invocation is delivered in the correct thread.
		 */
		object_.reset();
		object_.moveToThread(&thread_);

		thread_.start();

		object_.invokeMethod(&InvokedObject::method,
				     ConnectionTypeBlocking, 42);

		switch (object_.status()) {
		case InvokedObject::NoCall:
			cout << "Method not invoked for custom thread" << endl;
			return TestFail;
		case InvokedObject::InvalidThread:
			cout << "Method invoked in incorrect thread for custom thread" << endl;
			return TestFail;
		default:
			break;
		}

		if (object_.value() != 42) {
			cout << "Method invoked with incorrect value for custom thread" << endl;
			return TestFail;
		}

		/* Test that direct method invocation bypasses threads. */
		object_.reset();
		object_.invokeMethod(&InvokedObject::method,
				     ConnectionTypeDirect, 42);

		switch (object_.status()) {
		case InvokedObject::NoCall:
			cout << "Method not invoked for custom thread" << endl;
			return TestFail;
		case InvokedObject::CallReceived:
			cout << "Method invoked in incorrect thread for direct call" << endl;
			return TestFail;
		default:
			break;
		}

		if (object_.value() != 42) {
			cout << "Method invoked with incorrect value for direct call" << endl;
			return TestFail;
		}

		/*
		 * Test invoking a method that takes reference arguments. This
		 * targets compilation, there's no need to check runtime
		 * results.
		 */
		object_.invokeMethod(&InvokedObject::methodWithReference,
				     ConnectionTypeBlocking, 42);

		/* Test invoking a method that returns a value. */
		int ret = object_.invokeMethod(&InvokedObject::methodWithReturn,
					       ConnectionTypeBlocking);
		if (ret != 42) {
			cout << "Method invoked return incorrect value (" << ret
			     << ")" << endl;
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
	InvokedObject object_;
};

TEST_REGISTER(ObjectInvokeTest)
