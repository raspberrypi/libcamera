/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * message.cpp - Messages test
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "message.h"
#include "thread.h"
#include "test.h"
#include "utils.h"

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

	MessageReceiver()
		: status_(NoMessage)
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

		receiver.postMessage(utils::make_unique<Message>(Message::None));

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
