/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_list.cpp - ControlList tests
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

class ControlListTest : public CameraTest, public Test
{
public:
	ControlListTest()
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
		CameraControlValidator validator(camera_.get());
		ControlList list(controls::controls, &validator);

		/* Test that the list is initially empty. */
		if (!list.empty()) {
			cout << "List should to be empty" << endl;
			return TestFail;
		}

		if (list.size() != 0) {
			cout << "List should contain zero items" << endl;
			return TestFail;
		}

		if (list.get(controls::Brightness)) {
			cout << "List should not contain Brightness control" << endl;
			return TestFail;
		}

		unsigned int count = 0;
		for (auto iter = list.begin(); iter != list.end(); ++iter)
			count++;

		if (count != 0) {
			cout << "List iteration should not produce any item" << endl;
			return TestFail;
		}

		/*
		 * Set a control, and verify that the list now contains it, and
		 * nothing else.
		 */
		list.set(controls::Brightness, -0.5f);

		if (list.empty()) {
			cout << "List should not be empty" << endl;
			return TestFail;
		}

		if (list.size() != 1) {
			cout << "List should contain one item" << endl;
			return TestFail;
		}

		if (!list.get(controls::Brightness)) {
			cout << "List should contain Brightness control" << endl;
			return TestFail;
		}

		count = 0;
		for (auto iter = list.begin(); iter != list.end(); ++iter)
			count++;

		if (count != 1) {
			cout << "List iteration should produce one item" << endl;
			return TestFail;
		}

		if (list.get(controls::Brightness) != -0.5f) {
			cout << "Incorrest Brightness control value" << endl;
			return TestFail;
		}

		if (list.get(controls::Contrast)) {
			cout << "List should not contain Contract control" << endl;
			return TestFail;
		}

		/* Update the first control and set a second one. */
		list.set(controls::Brightness, 0.0f);
		list.set(controls::Contrast, 1.5f);

		if (!list.get(controls::Brightness) ||
		    !list.get(controls::Contrast)) {
			cout << "List should contain Brightness and Contrast controls"
			     << endl;
			return TestFail;
		}

		if (list.get(controls::Brightness) != 0.0f ||
		    list.get(controls::Contrast) != 1.5f) {
			cout << "Failed to retrieve control value" << endl;
			return TestFail;
		}

		/*
		 * Update both controls and verify that the container doesn't
		 * grow.
		 */
		list.set(controls::Brightness, 0.5f);
		list.set(controls::Contrast, 1.1f);

		if (list.get(controls::Brightness) != 0.5f ||
		    list.get(controls::Contrast) != 1.1f) {
			cout << "Failed to update control value" << endl;
			return TestFail;
		}

		if (list.size() != 2) {
			cout << "List should contain two elements" << endl;
			return TestFail;
		}

		/*
		 * Attempt to set an invalid control and verify that the
		 * operation failed.
		 */
		list.set(controls::AwbEnable, true);

		if (list.get(controls::AwbEnable)) {
			cout << "List shouldn't contain AwbEnable control" << endl;
			return TestFail;
		}

		/*
		 * Create a new list with a new control and merge it with the
		 * existing one, verifying that the existing controls
		 * values don't get overwritten.
		 */
		ControlList mergeList(controls::controls, &validator);
		mergeList.set(controls::Brightness, 0.7f);
		mergeList.set(controls::Saturation, 0.4f);

		mergeList.merge(list);
		if (mergeList.size() != 3) {
			cout << "Merged list should contain three elements" << endl;
			return TestFail;
		}

		if (list.size() != 2) {
			cout << "The list to merge should contain two elements"
			     << endl;
			return TestFail;
		}

		if (!mergeList.get(controls::Brightness) ||
		    !mergeList.get(controls::Contrast) ||
		    !mergeList.get(controls::Saturation)) {
			cout << "Merged list does not contain all controls" << endl;
			return TestFail;
		}

		if (mergeList.get(controls::Brightness) != 0.7f) {
			cout << "Brightness control value changed after merging lists"
			     << endl;
			return TestFail;
		}

		if (mergeList.get(controls::Contrast) != 1.1f) {
			cout << "Contrast control value changed after merging lists"
			     << endl;
			return TestFail;
		}

		if (mergeList.get(controls::Saturation) != 0.4f) {
			cout << "Saturation control value changed after merging lists"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlListTest)
