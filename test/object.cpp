/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * object.cpp - Object tests
 */

#include <iostream>

#include <libcamera/object.h>

#include "message.h"
#include "thread.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class InstrumentedObject : public Object
{
public:
	enum Status {
		NoMessage,
		MessageReceived,
	};

	InstrumentedObject()
		: status_(NoMessage)
	{
	}

	Status status() const { return status_; }
	void reset() { status_ = NoMessage; }

protected:
	void message(Message *msg) override
	{
		if (msg->type() == Message::ThreadMoveMessage)
			status_ = MessageReceived;

		Object::message(msg);
	}

private:
	Status status_;
};

class ObjectTest : public Test
{
protected:
	int init()
	{
		a_ = new InstrumentedObject();
		return TestPass;
	}

	int run()
	{
		/* Verify that moving an object to a different thread succeeds. */
		a_->moveToThread(&thread_);

		if (a_->thread() != &thread_ || a_->thread() == Thread::current()) {
			cout << "Failed to move object to thread" << endl;
			return TestFail;
		}

		/* Verify that objects receive a ThreadMoveMessage when moved. */
		if (a_->status() != InstrumentedObject::MessageReceived) {
			cout << "Moving object didn't deliver ThreadMoveMessage" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		delete a_;
	}

private:
	InstrumentedObject *a_;

	Thread thread_;
};

TEST_REGISTER(ObjectTest)
