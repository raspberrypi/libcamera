/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * process_test.cpp - Process test
 */

#include <iostream>
#include <unistd.h>
#include <vector>

#include <libcamera/event_dispatcher.h>
#include <libcamera/timer.h>

#include "process.h"
#include "test.h"
#include "thread.h"
#include "utils.h"

using namespace std;
using namespace libcamera;

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

		int ret = proc_.start("/proc/self/exe", args);
		if (ret) {
			cerr << "failed to start process" << endl;
			return TestFail;
		}

		timeout.start(100);
		while (timeout.isRunning())
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
	void procFinished(Process *proc, enum Process::ExitStatus exitStatus, int exitCode)
	{
		exitStatus_ = exitStatus;
		exitCode_ = exitCode;
	}

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

	return ProcessTest().execute();
}
