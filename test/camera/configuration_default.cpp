/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include "camera_test.h"

using namespace std;

namespace {

class ConfigurationDefault : public CameraTest
{
protected:
	int run()
	{
		CameraConfiguration conf;

		/* Test asking for configuration for a video stream. */
		conf = camera_->streamConfiguration({ Stream::VideoRecording() });
		if (!conf.isValid()) {
			cout << "Default configuration invalid" << endl;
			return TestFail;
		}

		/*
		 * Test that asking for configuration for an empty array of
		 * stream usages returns an empty list of configurations.
		 */
		conf = camera_->streamConfiguration({});
		if (conf.isValid()) {
			cout << "Failed to retrieve configuration for empty usage list"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

} /* namespace */

TEST_REGISTER(ConfigurationDefault);
