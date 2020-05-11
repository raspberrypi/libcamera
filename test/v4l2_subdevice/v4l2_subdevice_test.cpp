/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice_test.cpp - VIMC-based V4L2 subdevice test
 */

#include <iostream>
#include <string.h>
#include <sys/stat.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "v4l2_subdevice_test.h"

using namespace std;
using namespace libcamera;

/*
 * This test runs on vimc media device. For a description of vimc, in the
 * context of libcamera testing, please refer to
 * 'test/media_device/media_device_link_test.cpp' file.
 *
 * If the vimc module is not loaded, the test gets skipped.
 */

int V4L2SubdeviceTest::init()
{
	enumerator_ = DeviceEnumerator::create();
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
		cerr << "Unable to find \'vimc\' media device node" << endl;
		return TestSkip;
	}

	MediaEntity *videoEntity = media_->getEntityByName("Scaler");
	if (!videoEntity) {
		cerr << "Unable to find media entity 'Scaler'" << endl;
		return TestFail;
	}

	scaler_ = new V4L2Subdevice(videoEntity);
	if (scaler_->open()) {
		cerr << "Unable to open video subdevice "
		     << scaler_->entity()->deviceNode() << endl;
		return TestSkip;
	}

	return 0;
}

void V4L2SubdeviceTest::cleanup()
{
	delete scaler_;
}
