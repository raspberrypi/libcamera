/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_list.cpp - ControlList tests
 */

#include <iostream>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class ControlListTest : public Test
{
protected:
	int init()
	{
		cm_ = new CameraManager();

		if (cm_->start()) {
			cout << "Failed to start camera manager" << endl;
			return TestFail;
		}

		camera_ = cm_->get("VIMC Sensor B");
		if (!camera_) {
			cout << "Can not find VIMC camera" << endl;
			return TestSkip;
		}

		return TestPass;
	}

	int run()
	{
		ControlList list(camera_.get());

		/* Test that the list is initially empty. */
		if (!list.empty()) {
			cout << "List should to be empty" << endl;
			return TestFail;
		}

		if (list.size() != 0) {
			cout << "List should contain zero items" << endl;
			return TestFail;
		}

		if (list.contains(Brightness)) {
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
		list[Brightness] = 255;

		if (list.empty()) {
			cout << "List should not be empty" << endl;
			return TestFail;
		}

		if (list.size() != 1) {
			cout << "List should contain one item" << endl;
			return TestFail;
		}

		if (!list.contains(Brightness)) {
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

		if (list[Brightness].get<int32_t>() != 255) {
			cout << "Incorrest Brightness control value" << endl;
			return TestFail;
		}

		if (list.contains(Contrast)) {
			cout << "List should not contain Contract control" << endl;
			return TestFail;
		}

		/*
		 * Set a second control through ControlInfo and retrieve it
		 * through both controlId and ControlInfo.
		 */
		const ControlInfoMap &controls = camera_->controls();
		const ControlInfo *brightness = &controls.find(Brightness)->second;
		const ControlInfo *contrast = &controls.find(Contrast)->second;

		list[brightness] = 64;
		list[contrast] = 128;

		if (!list.contains(Contrast) || !list.contains(contrast)) {
			cout << "List should contain Contrast control" << endl;
			return TestFail;
		}

		/*
		 * Test control value retrieval and update through ControlInfo.
		 */
		if (list[brightness].get<int32_t>() != 64 ||
		    list[contrast].get<int32_t>() != 128) {
			cout << "Failed to retrieve control value" << endl;
			return TestFail;
		}

		list[brightness] = 10;
		list[contrast] = 20;

		if (list[brightness].get<int32_t>() != 10 ||
		    list[contrast].get<int32_t>() != 20) {
			cout << "Failed to update control value" << endl;
			return TestFail;
		}

		/*
		 * Assert that the container has not grown with the control
		 * updated.
		 */
		if (list.size() != 2) {
			cout << "List should contain two elements" << endl;
			return TestFail;
		}

		/*
		 * Test list merging. Create a new list, add two controls with
		 * one overlapping the existing list, merge the lists and clear
		 * the old list. Verify that the new list is empty and that the
		 * new list contains the expected items and values.
		 */
		ControlList newList(camera_.get());

		newList[Brightness] = 128;
		newList[Saturation] = 255;
		newList.update(list);

		list.clear();

		if (list.size() != 0) {
			cout << "Old List should contain zero items" << endl;
			return TestFail;
		}

		if (!list.empty()) {
			cout << "Old List should be empty" << endl;
			return TestFail;
		}

		if (newList.size() != 3) {
			cout << "New list has incorrect size" << endl;
			return TestFail;
		}

		if (!newList.contains(Brightness) ||
		    !newList.contains(Contrast) ||
		    !newList.contains(Saturation)) {
			cout << "New list contains incorrect items" << endl;
			return TestFail;
		}

		if (newList[Brightness].get<int32_t>() != 10 ||
		    newList[Contrast].get<int32_t>() != 20 ||
		    newList[Saturation].get<int32_t>() != 255) {
			cout << "New list contains incorrect values" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		if (camera_) {
			camera_->release();
			camera_.reset();
		}

		cm_->stop();
		delete cm_;
	}

private:
	CameraManager *cm_;
	std::shared_ptr<Camera> camera_;
};

TEST_REGISTER(ControlListTest)
