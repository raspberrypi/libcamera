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

class ConfigurationSet : public CameraTest
{
protected:
	int run()
	{
		CameraConfiguration config =
			camera_->generateConfiguration({ Stream::VideoRecording() });
		StreamConfiguration *cfg = &config[config.front()];

		if (!config.isValid()) {
			cout << "Failed to read default configuration" << endl;
			return TestFail;
		}

		if (camera_->acquire()) {
			cout << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		/* Test that setting the default configuration works. */
		if (camera_->configure(config)) {
			cout << "Failed to set default configuration" << endl;
			return TestFail;
		}

		/*
		 * Test that configuring the camera fails if it is not
		 * acquired, this will also test release and reacquiring
		 * of the camera.
		 */
		if (camera_->release()) {
			cout << "Failed to release the camera" << endl;
			return TestFail;
		}

		if (!camera_->configure(config)) {
			cout << "Setting configuration on a camera not acquired succeeded when it should have failed"
			     << endl;
			return TestFail;
		}

		if (camera_->acquire()) {
			cout << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		/*
		 * Test that modifying the default configuration works. Doubling
		 * the default configuration of the VIMC camera is known to
		 * work.
		 */
		cfg->size.width *= 2;
		cfg->size.height *= 2;
		if (camera_->configure(config)) {
			cout << "Failed to set modified configuration" << endl;
			return TestFail;
		}

		/*
		 * Test that setting an invalid configuration fails.
		 */
		cfg->size = { 0, 0 };
		if (!camera_->configure(config)) {
			cout << "Invalid configuration incorrectly accepted" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

} /* namespace */

TEST_REGISTER(ConfigurationSet);
