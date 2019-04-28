/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event.cpp - Event test
 */

#include <iostream>
#include <string.h>
#include <unistd.h>

#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>
#include <libcamera/event_notifier.h>
#include <libcamera/timer.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class EventTest : public Test
{
protected:
	void readReady(EventNotifier *notifier)
	{
		size_ = read(notifier->fd(), data_, sizeof(data_));
		notified_ = true;
	}

	int init()
	{
		return pipe(pipefd_);
	}

	int run()
	{
		EventDispatcher *dispatcher = CameraManager::instance()->eventDispatcher();
		std::string data("H2G2");
		Timer timeout;
		ssize_t ret;

		EventNotifier readNotifier(pipefd_[0], EventNotifier::Read);
		readNotifier.activated.connect(this, &EventTest::readReady);

		/* Test read notification with data. */
		memset(data_, 0, sizeof(data_));
		size_ = 0;

		ret = write(pipefd_[1], data.data(), data.size());
		if (ret < 0) {
			cout << "Pipe write failed" << endl;
			return TestFail;
		}

		timeout.start(100);
		dispatcher->processEvents();
		timeout.stop();

		if (static_cast<size_t>(size_) != data.size()) {
			cout << "Event notifier read ready test failed" << endl;
			return TestFail;
		}

		/* Test read notification without data. */
		notified_ = false;

		timeout.start(100);
		dispatcher->processEvents();
		timeout.stop();

		if (notified_) {
			cout << "Event notifier read no ready test failed" << endl;
			return TestFail;
		}

		/* Test read notifier disabling. */
		notified_ = false;
		readNotifier.setEnabled(false);

		ret = write(pipefd_[1], data.data(), data.size());
		if (ret < 0) {
			cout << "Pipe write failed" << endl;
			return TestFail;
		}

		timeout.start(100);
		dispatcher->processEvents();
		timeout.stop();

		if (notified_) {
			cout << "Event notifier read disabling failed" << endl;
			return TestFail;
		}

		/* Test read notifier enabling. */
		notified_ = false;
		readNotifier.setEnabled(true);

		timeout.start(100);
		dispatcher->processEvents();
		timeout.stop();

		if (!notified_) {
			cout << "Event notifier read enabling test failed" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		close(pipefd_[0]);
		close(pipefd_[1]);
	}

private:
	int pipefd_[2];

	bool notified_;
	char data_[16];
	ssize_t size_;
};

TEST_REGISTER(EventTest)
