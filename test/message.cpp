/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Messages test
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/base/message.h>
#include <libcamera/base/object.h>
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

		MessageReceiver *receiver = new MessageReceiver();
		receiver->moveToThread(&thread_);

		thread_.start();

		receiver->postMessage(std::make_unique<Message>(Message::None));

		this_thread::sleep_for(chrono::milliseconds(100));

		MessageReceiver::Status status = receiver->status();
		receiver->deleteLater();

		switch (status) {
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
		 * Test recursive calls to Thread::dispatchMessages(). Messages
		 * should be delivered correctly, without crashes or memory
		 * leaks. Two messages need to be posted to ensure we don't only
		 * test the simple case of a queue containing a single message.
		 */
		RecursiveMessageReceiver *recursiveReceiver = new RecursiveMessageReceiver();
		recursiveReceiver->moveToThread(&thread_);

		recursiveReceiver->postMessage(std::make_unique<Message>(Message::None));
		recursiveReceiver->postMessage(std::make_unique<Message>(Message::UserMessage));

		this_thread::sleep_for(chrono::milliseconds(10));

		bool success = recursiveReceiver->success();
		recursiveReceiver->deleteLater();

		if (!success) {
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
