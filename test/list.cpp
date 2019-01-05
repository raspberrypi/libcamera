/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * list.cpp - camera list tests
 */

#include <iostream>

#include <libcamera/camera_manager.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class ListTest : public Test
{
protected:
	int init()
	{
		cm = CameraManager::instance();
		cm->start();

		return 0;
	}

	int run()
	{
		unsigned int count = 0;

		for (auto name : cm->list()) {
			cout << "- " << name << endl;
			count++;
		}

		return count ? 0 : -ENODEV;
	}

	void cleanup()
	{
		cm->stop();
	}

private:
	CameraManager *cm;
};

TEST_REGISTER(ListTest)
