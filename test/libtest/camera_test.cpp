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

CameraTest::CameraTest(const char *name, bool isolate)
{
	cm_ = new CameraManager();

	if (isolate)
		setenv("LIBCAMERA_IPA_FORCE_ISOLATION", "1", 1);

	if (cm_->start()) {
		cerr << "Failed to start camera manager" << endl;
		status_ = TestFail;
		return;
	}

	camera_ = cm_->get(name);
	if (!camera_) {
		cerr << "Can not find '" << name << "' camera" << endl;
		status_ = TestSkip;
		return;
	}

	/* Sanity check that the camera has streams. */
	if (camera_->streams().empty()) {
		cerr << "Camera has no stream" << endl;
		status_ = TestFail;
		return;
	}

	status_ = TestPass;
}

CameraTest::~CameraTest()
{
	if (camera_) {
		camera_->release();
		camera_.reset();
	}

	cm_->stop();
	delete cm_;
}
