/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Object tests
 */

#include <iostream>

#include <libcamera/base/message.h>
#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

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

	InstrumentedObject(Object *parent = nullptr)
		: Object(parent), status_(NoMessage)
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
		/*
		 * Create a hierarchy of objects:
		 * A -> B -> C
		 *   \->D
		 * E
		 */
		a_ = new InstrumentedObject();
		b_ = new InstrumentedObject(a_);
		c_ = new InstrumentedObject(b_);
		d_ = new InstrumentedObject(a_);
		e_ = new InstrumentedObject();
		f_ = nullptr;

		return TestPass;
	}

	int run()
	{
		/* Verify the parent-child relationships. */
		if (a_->parent() != nullptr || b_->parent() != a_ ||
		    c_->parent() != b_ || d_->parent() != a_ ||
		    e_->parent() != nullptr) {
			cout << "Incorrect parent-child relationships" << endl;
			return TestFail;
		}

		/*
		 * Verify that moving an object with no parent to a different
		 * thread succeeds.
		 */
		e_->moveToThread(&thread_);

		if (e_->thread() != &thread_ || e_->thread() == Thread::current()) {
			cout << "Failed to move object to thread" << endl;
			return TestFail;
		}

		/*
		 * Verify that moving an object with a parent to a different
		 * thread fails. This results in an undefined behaviour, the
		 * test thus depends on the internal implementation returning
		 * without performing any change.
		 */
		b_->moveToThread(&thread_);

		if (b_->thread() != Thread::current()) {
			cout << "Moving object with parent to thread shouldn't succeed" << endl;
			return TestFail;
		}

		/*
		 * Verify that moving an object with children to a different
		 * thread moves all the children.
		 */
		a_->moveToThread(&thread_);

		if (a_->thread() != &thread_ || b_->thread() != &thread_ ||
		    c_->thread() != &thread_ || d_->thread() != &thread_) {
			cout << "Failed to move children to thread" << endl;
			return TestFail;
		}

		/* Verify that objects are bound to the thread of their parent. */
		f_ = new InstrumentedObject(d_);

		if (f_->thread() != &thread_) {
			cout << "Failed to bind child to parent thread" << endl;
			return TestFail;
		}

		/* Verify that objects receive a ThreadMoveMessage when moved. */
		if (a_->status() != InstrumentedObject::MessageReceived ||
		    b_->status() != InstrumentedObject::MessageReceived ||
		    c_->status() != InstrumentedObject::MessageReceived ||
		    d_->status() != InstrumentedObject::MessageReceived ||
		    e_->status() != InstrumentedObject::MessageReceived) {
			cout << "Moving object didn't deliver ThreadMoveMessage" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		delete a_;
		delete b_;
		delete c_;
		delete d_;
		delete e_;
		delete f_;
	}

private:
	InstrumentedObject *a_;
	InstrumentedObject *b_;
	InstrumentedObject *c_;
	InstrumentedObject *d_;
	InstrumentedObject *e_;
	InstrumentedObject *f_;

	Thread thread_;
};

TEST_REGISTER(ObjectTest)
