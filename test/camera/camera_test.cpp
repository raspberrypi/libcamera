/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include "camera_test.h"

using namespace libcamera;
using namespace std;

int CameraTest::init()
{
	cm_ = CameraManager::instance();

	if (cm_->start()) {
		cout << "Failed to start camera manager" << endl;
		return TestFail;
	}

	camera_ = cm_->get("VIMC Sensor B");
	if (!camera_) {
		cout << "Can not find VIMC camera" << endl;
		return TestSkip;
	}

	/* Sanity check that the camera has streams. */
	if (camera_->streams().empty()) {
		cout << "Camera has no stream" << endl;
		return TestFail;
	}

	return TestPass;
}

void CameraTest::cleanup()
{
	if (camera_) {
		camera_->release();
		camera_.reset();
	}

	cm_->stop();
};

bool CameraTest::configurationValid(const std::set<Stream *> &streams,
				    const std::map<Stream *, StreamConfiguration> &conf) const
{
	/* Test that the numbers of streams matches that of configuration. */
	if (streams.size() != conf.size())
		return false;

	/*
	 * Test that stream can be found in configuration and that the
	 * configuration is valid.
	 */
	for (Stream *stream : streams) {
		std::map<Stream *, StreamConfiguration>::const_iterator it =
			conf.find(stream);

		if (it == conf.end())
			return false;

		const StreamConfiguration *sconf = &it->second;
		if (sconf->width == 0 || sconf->height == 0 ||
		    sconf->pixelFormat == 0 || sconf->bufferCount == 0)
			return false;
	}

	return true;
}
