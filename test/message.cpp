/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * message.cpp - Messages test
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/base/message.h>
#include <libcamera/base/thread.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class MessageReceiver : public Object
{
public:
	enum Status {
		NoMessage,
		InvalidThread,
		MessageReceived,
	};

	MessageReceiver(Object *parent = nullptr)
		: Object(parent), status_(NoMessage)
	{
	}

	Status status() const { return status_; }
	void reset() { status_ = NoMessage; }

protected:
	void message(Message *msg)
	{
		if (msg->type() != Message::None) {
			Object::message(msg);
			return;
		}

		if (thread() != Thread::current())
			status_ = InvalidThread;
		else
			status_ = MessageReceived;
	}

private:
	Status status_;
};

class RecursiveMessageReceiver : public Object
{
public:
	RecursiveMessageReceiver()
		: child_(this), success_(false)
	{
	}

	bool success() const { return success_; }

protected:
	void message([[maybe_unused]] Message *msg)
	{
		if (msg->type() != Message::None) {
			Object::message(msg);
			return;
		}

		child_.postMessage(std::make_unique<Message>(Message::None));

		/*
		 * If the child has already received the message, something is
		 * wrong.
		 */
		if (child_.status() != MessageReceiver::NoMessage)
			return;

		Thread::current()->dispatchMessages(Message::None);

		/* The child should now have received the message. */
		if (child_.status() == MessageReceiver::MessageReceived)
			success_ = true;
	}

private:
	MessageReceiver child_;
	bool success_;
};

class SlowMessageReceiver : public Object
{
protected:
	void message(Message *msg)
	{
		if (msg->type() != Message::None) {
			Object::message(msg);
			return;
		}

		/*
		 * Don't access any member of the object here (including the
		 * vtable) as the object will be deleted by the main thread
		 * while we're sleeping.
		 */
		this_thread::sleep_for(chrono::milliseconds(100));
	}
};

class MessageTest : public Test
{
protected:
	int run()
	{
		Message::Type msgType[2] = {
			Message::registerMessageType(),
			Message::registerMessageType(),
		};

		if (msgType[0] != Message::UserMessage ||
		    msgType[1] != Message::UserMessage + 1) {
			cout << "Failed to register message types" << endl;
			return TestFail;
		}

		MessageReceiver receiver;
		receiver.moveToThread(&thread_);

		thread_.start();

		receiver.postMessage(std::make_unique<Message>(Message::None));

		this_thread::sleep_for(chrono::milliseconds(100));

		switch (receiver.status()) {
		case MessageReceiver::NoMessage:
			cout << "No message received" << endl;
			return TestFail;
		case MessageReceiver::InvalidThread:
			cout << "Message received in incorrect thread" << endl;
			return TestFail;
		default:
			break;
		}

		/*
		 * Test for races between message delivery and object deletion.
		 * Failures result in assertion errors, there is no need for
		 * explicit checks.
		 */
		SlowMessageReceiver *slowReceiver = new SlowMessageReceiver();
		slowReceiver->moveToThread(&thread_);
		slowReceiver->postMessage(std::make_unique<Message>(Message::None));

		this_thread::sleep_for(chrono::milliseconds(10));

		delete slowReceiver;

		this_thread::sleep_for(chrono::milliseconds(100));

		/*
		 * Test recursive calls to Thread::dispatchMessages(). Messages
		 * should be delivered correctly, without crashes or memory
		 * leaks. Two messages need to be posted to ensure we don't only
		 * test the simple case of a queue containing a single message.
		 */
		std::unique_ptr<RecursiveMessageReceiver> recursiveReceiver =
			std::make_unique<RecursiveMessageReceiver>();
		recursiveReceiver->moveToThread(&thread_);

		recursiveReceiver->postMessage(std::make_unique<Message>(Message::None));
		recursiveReceiver->postMessage(std::make_unique<Message>(Message::UserMessage));

		this_thread::sleep_for(chrono::milliseconds(10));

		if (!recursiveReceiver->success()) {
			cout << "Recursive message delivery failed" << endl;
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

TEST_REGISTER(MessageTest)
