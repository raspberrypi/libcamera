/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * log.cpp - log API test
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

#include <libcamera/logging.h>

#include "log.h"
#include "test.h"

using namespace std;
using namespace libcamera;

LOG_DEFINE_CATEGORY(LogAPITest)

class LogAPITest : public Test
{
protected:
	int run() override
	{
		int fd = open("/tmp", O_TMPFILE | O_RDWR, S_IRUSR | S_IWUSR);
		if (fd < 0) {
			cerr << "Failed to open tmp log file" << endl;
			return TestFail;
		}

		char path[32];
		snprintf(path, sizeof(path), "/proc/self/fd/%u", fd);

		logSetFile(path);

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

		char buf[1000];
		memset(buf, 0, sizeof(buf));
		lseek(fd, 0, SEEK_SET);
		if (read(fd, buf, sizeof(buf)) < 0) {
			cerr << "Failed to read tmp log file" << endl;
			return TestFail;
		}
		close(fd);

		std::list<int> goodList = { 1, 3, 5 };
		std::basic_istringstream<char> iss((std::string(buf)));
		std::string line;
		while (getline(iss, line)) {
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
};

TEST_REGISTER(LogAPITest)
