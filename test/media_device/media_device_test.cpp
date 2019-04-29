/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * media_device_test.cpp - libcamera media device test base class
 */

#include <iostream>

#include "media_device_test.h"

using namespace libcamera;
using namespace std;

int MediaDeviceTest::init()
{
	enumerator_ = unique_ptr<DeviceEnumerator>(DeviceEnumerator::create());
	if (!enumerator_) {
		cerr << "Failed to create device enumerator" << endl;
		return TestFail;
	}

	if (enumerator_->enumerate()) {
		cerr << "Failed to enumerate media devices" << endl;
		return TestFail;
	}

	DeviceMatch dm("vimc");
	media_ = enumerator_->search(dm);
	if (!media_) {
		cerr << "No VIMC media device found: skip test" << endl;
		return TestSkip;
	}

	return TestPass;
}
