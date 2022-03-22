/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * process_test.cpp - Process test
 */

#include <iostream>
#include <unistd.h>
#include <vector>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/process.h"

#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

class ProcessTestChild
{
public:
	int run(int status)
	{
		usleep(50000);

		return status;
	}
};

class ProcessTest : public Test
{
public:
	ProcessTest()
		: exitStatus_(Process::NotExited), exitCode_(-1)
	{
	}

protected:
	int run()
	{
		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		Timer timeout;

		int exitCode = 42;
		vector<std::string> args;
		args.push_back(to_string(exitCode));
		proc_.finished.connect(this, &ProcessTest::procFinished);

		/* Test that kill() on an unstarted process is safe. */
		proc_.kill();

		/* Test starting the process and retrieving the exit code. */
		int ret = proc_.start(self(), args);
		if (ret) {
			cerr << "failed to start process" << endl;
			return TestFail;
		}

		timeout.start(2000ms);
		while (timeout.isRunning() && exitStatus_ == Process::NotExited)
			dispatcher->processEvents();

		if (exitStatus_ != Process::NormalExit) {
			cerr << "process did not exit normally" << endl;
			return TestFail;
		}

		if (exitCode != exitCode_) {
			cerr << "exit code should be " << exitCode
			     << ", actual is " << exitCode_ << endl;
			return TestFail;
		}

		return TestPass;
	}

private:
	void procFinished(enum Process::ExitStatus exitStatus, int exitCode)
	{
		exitStatus_ = exitStatus;
		exitCode_ = exitCode;
	}

	ProcessManager processManager_;

	Process proc_;
	enum Process::ExitStatus exitStatus_;
	int exitCode_;
};

/*
 * Can't use TEST_REGISTER() as single binary needs to act as both
 * parent and child processes.
 */
int main(int argc, char **argv)
{
	if (argc == 2) {
		int status = std::stoi(argv[1]);
		ProcessTestChild child;
		return child.run(status);
	}

	ProcessTest test;
	test.setArgs(argc, argv);
	return test.execute();
}
