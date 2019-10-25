/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_info.cpp - ControlInfoMap tests
 */

#include <iostream>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "camera_controls.h"

#include "camera_test.h"
#include "test.h"

using namespace std;
using namespace libcamera;

class ControlInfoMapTest : public CameraTest, public Test
{
public:
	ControlInfoMapTest()
		: CameraTest("VIMC Sensor B")
	{
	}

protected:
	int init() override
	{
		return status_;
	}

	int run() override
	{
		const ControlInfoMap &info = camera_->controls();

		/* Test looking up a valid control by ControlId. */
		if (info.count(&controls::Brightness) != 1) {
			cerr << "count() on valid control failed" << endl;
			return TestFail;
		}

		if (info.find(&controls::Brightness) == info.end()) {
			cerr << "find() on valid control failed" << endl;
			return TestFail;
		}

		info.at(&controls::Brightness);

		/* Test looking up a valid control by numerical ID. */
		if (info.count(controls::Brightness.id()) != 1) {
			cerr << "count() on valid ID failed" << endl;
			return TestFail;
		}

		if (info.find(controls::Brightness.id()) == info.end()) {
			cerr << "find() on valid ID failed" << endl;
			return TestFail;
		}

		info.at(controls::Brightness.id());

		/* Test looking up an invalid control by numerical ID. */
		if (info.count(12345) != 0) {
			cerr << "count() on invalid ID failed" << endl;
			return TestFail;
		}

		if (info.find(12345) != info.end()) {
			cerr << "find() on invalid ID failed" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlInfoMapTest)
