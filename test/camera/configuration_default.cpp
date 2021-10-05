/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std;

namespace {

class ConfigurationDefault : public CameraTest, public Test
{
public:
	ConfigurationDefault()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	int init() override
	{
		return status_;
	}

	int run() override
	{
		std::unique_ptr<CameraConfiguration> config;

		/* Test asking for configuration for a video stream. */
		config = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!config || config->size() != 1) {
			cout << "Default configuration invalid" << endl;
			return TestFail;
		}

		/*
		 * Test that asking for configuration for an empty array of
		 * stream roles returns an empty camera configuration.
		 */
		config = camera_->generateConfiguration({});
		if (!config || config->size() != 0) {
			cout << "Failed to retrieve configuration for empty roles list"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

} /* namespace */

TEST_REGISTER(ConfigurationDefault)
