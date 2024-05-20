/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ControlInfoMap tests
 */

#include <iostream>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "libcamera/internal/camera_controls.h"

#include "camera_test.h"
#include "test.h"

using namespace std;
using namespace libcamera;

class ControlInfoMapTest : public CameraTest, public Test
{
public:
	ControlInfoMapTest()
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
		const ControlInfoMap &infoMap = camera_->controls();

		/* Test looking up a valid control by ControlId. */
		if (infoMap.count(&controls::Brightness) != 1) {
			cerr << "count() on valid control failed" << endl;
			return TestFail;
		}

		if (infoMap.find(&controls::Brightness) == infoMap.end()) {
			cerr << "find() on valid control failed" << endl;
			return TestFail;
		}

		infoMap.at(&controls::Brightness);

		/* Test looking up a valid control by numerical ID. */
		if (infoMap.count(controls::Brightness.id()) != 1) {
			cerr << "count() on valid ID failed" << endl;
			return TestFail;
		}

		if (infoMap.find(controls::Brightness.id()) == infoMap.end()) {
			cerr << "find() on valid ID failed" << endl;
			return TestFail;
		}

		infoMap.at(controls::Brightness.id());

		/* Test looking up an invalid control by numerical ID. */
		if (infoMap.count(12345) != 0) {
			cerr << "count() on invalid ID failed" << endl;
			return TestFail;
		}

		if (infoMap.find(12345) != infoMap.end()) {
			cerr << "find() on invalid ID failed" << endl;
			return TestFail;
		}

		/* Test looking up a control on a default-constructed infoMap */
		const ControlInfoMap emptyInfoMap;
		if (emptyInfoMap.find(12345) != emptyInfoMap.end()) {
			cerr << "find() on empty ControlInfoMap failed" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlInfoMapTest)
