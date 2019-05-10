/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * load-so.cpp - loading .so tests
 */

#include <iostream>
#include <string.h>

#include "ipa_module.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class IPAModuleTest : public Test
{
protected:
	int runTest(const string &path, const struct IPAModuleInfo &testInfo)
	{
		int ret = 0;

		IPAModule *ll = new IPAModule(path);

		if (!ll->isValid()) {
			cerr << "test IPA module " << path << " is invalid"
			     << endl;
			delete ll;
			return -1;
		}

		const struct IPAModuleInfo &info = ll->info();

		if (strcmp(info.name, testInfo.name)) {
			cerr << "test IPA module has incorrect name" << endl;
			cerr << "expected \"" << testInfo.name << "\", got \""
			     << info.name << "\"" << endl;
			ret = -1;
		}

		if (info.version != testInfo.version) {
			cerr << "test IPA module has incorrect version" << endl;
			cerr << "expected \"" << testInfo.version << "\", got \""
			     << info.version << "\"" << endl;
			ret = -1;
		}

		delete ll;
		return ret;
	}

	int run() override
	{
		int count = 0;

		const struct IPAModuleInfo testInfo = {
			"It's over nine thousand!",
			9001,
		};

		count += runTest("test/ipa/ipa-dummy-c.so", testInfo);

		count += runTest("test/ipa/ipa-dummy-cpp.so", testInfo);

		if (count < 0)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(IPAModuleTest)
