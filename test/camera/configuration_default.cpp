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
		std::map<Stream *, StreamConfiguration> conf;

		/*
		 * Test that asking for default configuration for a valid
		 * array of streams returns something valid.
		 */
		std::set<Stream *> streams = { *camera_->streams().begin() };
		conf = camera_->streamConfiguration(streams);
		if (conf.empty()) {
			cout << "Failed to retrieve configuration for valid streams"
			     << endl;
			return TestFail;
		}

		if (!configurationValid(streams, conf)) {
			cout << "Default configuration invalid" << endl;
			return TestFail;
		}

		/*
		 * Test that asking for configuration for an empty array of
		 * streams returns an empty list of configurations.
		 */
		std::set<Stream *> streams_empty = {};
		conf = camera_->streamConfiguration(streams_empty);
		if (!conf.empty()) {
			cout << "Failed to retrieve configuration for empty streams"
			     << endl;
			return TestFail;
		}

		/*
		 * Test that asking for configuration for an array of bad streams
		 * returns an empty list of configurations.
		 */
		Stream *stream_bad = reinterpret_cast<Stream *>(0xdeadbeef);
		std::set<Stream *> streams_bad = { stream_bad };
		conf = camera_->streamConfiguration(streams_bad);
		if (!conf.empty()) {
			cout << "Failed to retrieve configuration for bad streams"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

} /* namespace */

TEST_REGISTER(ConfigurationDefault);
