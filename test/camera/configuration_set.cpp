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

class ConfigurationSet : public CameraTest, public Test
{
public:
	ConfigurationSet()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	int init() override
	{
		if (status_ != TestPass)
			return status_;

		config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!config_ || config_->size() != 1) {
			cout << "Failed to generate default configuration" << endl;
			return TestFail;
		}

		return TestPass;
	}

	int run() override
	{
		StreamConfiguration &cfg = config_->at(0);

		if (camera_->acquire()) {
			cout << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		/* Test that setting the default configuration works. */
		if (camera_->configure(config_.get())) {
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

		if (!camera_->configure(config_.get())) {
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
		cfg.size.width *= 2;
		cfg.size.height *= 2;
		if (camera_->configure(config_.get())) {
			cout << "Failed to set modified configuration" << endl;
			return TestFail;
		}

		/*
		 * Test that setting an invalid configuration fails.
		 */
		cfg.size = { 0, 0 };
		if (!camera_->configure(config_.get())) {
			cout << "Invalid configuration incorrectly accepted" << endl;
			return TestFail;
		}

		return TestPass;
	}

	std::unique_ptr<CameraConfiguration> config_;
};

} /* namespace */

TEST_REGISTER(ConfigurationSet)
