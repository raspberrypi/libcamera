/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Object deletion tests
 */

#include <iostream>

#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class TestObject : public Object
{
public:
	TestObject(unsigned int *count)
		: deleteCount_(count)
	{
	}

	~TestObject()
	{
		/* Count the deletions from the correct thread. */
		if (thread() == Thread::current())
			(*deleteCount_)++;
	}

	unsigned int *deleteCount_;
};

class DeleterThread : public Thread
{
public:
	DeleterThread(Object *obj)
		: object_(obj)
	{
	}

protected:
	void run()
	{
		object_->deleteLater();
	}

private:
	Object *object_;
};

class ObjectDeleteTest : public Test
{
protected:
	int run()
	{
		/*
		 * Test that deferred deletion is executed from the object's
		 * thread, not the caller's thread.
		 */
		unsigned int count = 0;
		TestObject *obj = new TestObject(&count);

		DeleterThread delThread(obj);
		delThread.start();
		delThread.wait();

		Thread::current()->dispatchMessages(Message::Type::DeferredDelete);

		if (count != 1) {
			cout << "Failed to dispatch DeferredDelete (" << count << ")" << endl;
			return TestFail;
		}

		/*
		 * Test that multiple calls to deleteLater() delete the object
		 * once only.
		 */
		count = 0;
		obj = new TestObject(&count);
		obj->deleteLater();
		obj->deleteLater();

		Thread::current()->dispatchMessages(Message::Type::DeferredDelete);
		if (count != 1) {
			cout << "Multiple deleteLater() failed (" << count << ")" << endl;
			return TestFail;
		}

		/*
		 * Test that deleteLater() works properly when called just
		 * before the object's thread exits.
		 */
		Thread boundThread;
		boundThread.start();

		count = 0;
		obj = new TestObject(&count);
		obj->moveToThread(&boundThread);

		obj->deleteLater();
		boundThread.exit();
		boundThread.wait();

		if (count != 1) {
			cout << "Object deletion right before thread exit failed (" << count << ")" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ObjectDeleteTest)
