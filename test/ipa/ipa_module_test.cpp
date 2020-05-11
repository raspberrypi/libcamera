/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module_test.cpp - Test loading of the VIMC IPA module and verify its info
 */

#include <iostream>
#include <string.h>

#include "libcamera/internal/ipa_module.h"

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

		if (memcmp(&info, &testInfo, sizeof(info))) {
			cerr << "IPA module information mismatch: expected:" << endl
			     << "moduleAPIVersion = "     << testInfo.moduleAPIVersion << endl
			     << "pipelineVersion = "      << testInfo.pipelineVersion << endl
			     << "pipelineName = "         << testInfo.pipelineName << endl
			     << "name = "                 << testInfo.name
			     << "got: " << endl
			     << "moduleAPIVersion = "     << info.moduleAPIVersion << endl
			     << "pipelineVersion = "      << info.pipelineVersion << endl
			     << "pipelineName = "         << info.pipelineName << endl
			     << "name = "                 << info.name << endl;
		}

		delete ll;
		return ret;
	}

	int run() override
	{
		int count = 0;

		const struct IPAModuleInfo testInfo = {
			IPA_MODULE_API_VERSION,
			0,
			"PipelineHandlerVimc",
			"vimc",
		};

		count += runTest("src/ipa/vimc/ipa_vimc.so", testInfo);

		if (count < 0)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(IPAModuleTest)
