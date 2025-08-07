/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * log API test
 */

#include <algorithm>
#include <fcntl.h>
#include <iostream>
#include <list>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include <libcamera/logging.h>

#include "test.h"

using namespace std;
using namespace libcamera;

LOG_DEFINE_CATEGORY(LogAPITest)
LOG_DEFINE_CATEGORY(Cat0)
LOG_DEFINE_CATEGORY(Cat1)
LOG_DEFINE_CATEGORY(Cat2)
LOG_DEFINE_CATEGORY(Cat3)
LOG_DEFINE_CATEGORY(Cat4)

class LogAPITest : public Test
{
protected:
	void doLogging()
	{
		logSetLevel("LogAPITest", "DEBUG");
		LOG(LogAPITest, Info) << "good 1";

		logSetLevel("LogAPITest", "WARN");
		LOG(LogAPITest, Info) << "bad";

		logSetLevel("LogAPITest", "ERROR");
		LOG(LogAPITest, Error) << "good 3";
		LOG(LogAPITest, Info) << "bad";

		logSetLevel("LogAPITest", "WARN");
		LOG(LogAPITest, Warning) << "good 5";
		LOG(LogAPITest, Info) << "bad";
	}

	int verifyOutput(istream &is)
	{
		list<int> goodList = { 1, 3, 5 };
		string line;
		while (getline(is, line)) {
			if (goodList.empty()) {
				cout << "Too many log lines" << endl;
				return TestFail;
			}

			unsigned int digit = line.back() - '0';
			unsigned int expect = goodList.front();
			goodList.pop_front();
			if (digit != expect) {
				cout << "Incorrect log line" << endl;
				return TestFail;
			}
		}

		if (!goodList.empty()) {
			cout << "Too few log lines" << endl;
			return TestFail;
		}

		return TestPass;
	}

	int testEnvLevels()
	{
		setenv("LIBCAMERA_LOG_LEVELS",
		       "Cat0:0,Cat0:9999,Cat1:INFO,Cat1:INVALID,Cat2:2,Cat2:-1,"
		       "Cat3:ERROR,Cat3:{[]},Cat4:4,Cat4:rubbish",
		       true);
		logSetTarget(libcamera::LoggingTargetNone);

		const std::pair<const LogCategory &, libcamera::LogSeverity> expected[] = {
			{ _LOG_CATEGORY(Cat0)(), libcamera::LogDebug },
			{ _LOG_CATEGORY(Cat1)(), libcamera::LogInfo },
			{ _LOG_CATEGORY(Cat2)(), libcamera::LogWarning },
			{ _LOG_CATEGORY(Cat3)(), libcamera::LogError },
			{ _LOG_CATEGORY(Cat4)(), libcamera::LogFatal },
		};
		bool ok = true;

		for (const auto &[c, s] : expected) {
			if (c.severity() != s) {
				ok = false;
				cerr << "Severity of " << c.name() << " (" << c.severity() << ") "
				     << "does not equal " << s << endl;
			}
		}

		return ok ? TestPass : TestFail;
	}

	int testFile()
	{
		int fd = open("/tmp", O_TMPFILE | O_RDWR, S_IRUSR | S_IWUSR);
		if (fd < 0) {
			cerr << "Failed to open tmp log file" << endl;
			return TestFail;
		}

		char path[32];
		snprintf(path, sizeof(path), "/proc/self/fd/%u", fd);

		if (logSetFile(path) < 0) {
			cerr << "Failed to set log file" << endl;
			close(fd);
			return TestFail;
		}

		doLogging();

		char buf[1000];
		memset(buf, 0, sizeof(buf));
		lseek(fd, 0, SEEK_SET);
		if (read(fd, buf, sizeof(buf)) < 0) {
			cerr << "Failed to read tmp log file" << endl;
			close(fd);
			return TestFail;
		}
		close(fd);

		istringstream iss(buf);
		return verifyOutput(iss);
	}

	int testStream()
	{
		stringstream log;
		/* Never fails, so no need to check return value */
		logSetStream(&log);

		doLogging();

		return verifyOutput(log);
	}

	int testTarget()
	{
		logSetTarget(LoggingTargetNone);
		logSetLevel("LogAPITest", "DEBUG");
		LOG(LogAPITest, Info) << "don't crash please";

		if (!logSetTarget(LoggingTargetFile))
			return TestFail;

		if (!logSetTarget(LoggingTargetStream))
			return TestFail;

		return TestPass;
	}

	int run() override
	{
		int ret = testEnvLevels();
		if (ret != TestPass)
			return TestFail;

		ret = testFile();
		if (ret != TestPass)
			return TestFail;

		ret = testStream();
		if (ret != TestPass)
			return TestFail;

		ret = testTarget();
		if (ret != TestPass)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(LogAPITest)
