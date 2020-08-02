/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipu3_pipeline_test.cpp - Intel IPU3 pipeline test
 */

#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"

#include "test.h"

using namespace std;
using namespace libcamera;

/*
 * Verify that the Intel IPU3 pipeline handler gets matched and cameras
 * are enumerated correctly.
 *
 * The test is supposed to be run on an IPU3 platform, otherwise it gets
 * skipped.
 *
 * The test lists all cameras registered in the system, if any camera is
 * available at all.
 */
class IPU3PipelineTest : public Test
{
protected:
	int init();
	int run();
	void cleanup();

private:
	CameraManager *cameraManager_;
	unsigned int sensors_;
};

int IPU3PipelineTest::init()
{
	unique_ptr<DeviceEnumerator> enumerator = DeviceEnumerator::create();
	if (!enumerator) {
		cerr << "Failed to create device enumerator" << endl;
		return TestFail;
	}

	if (enumerator->enumerate()) {
		cerr << "Failed to enumerate media devices" << endl;
		return TestFail;
	}

	DeviceMatch imgu_dm("ipu3-imgu");
	DeviceMatch cio2_dm("ipu3-cio2");

	if (!enumerator->search(imgu_dm)) {
		cerr << "Failed to find IPU3 IMGU: test skip" << endl;
		return TestSkip;
	}

	std::shared_ptr<MediaDevice> cio2 = enumerator->search(cio2_dm);
	if (!cio2) {
		cerr << "Failed to find IPU3 CIO2: test skip" << endl;
		return TestSkip;
	}

	/*
	 * Camera sensor are connected to the CIO2 unit.
	 * Count how many sensors are connected in the system
	 * and later verify this matches the number of registered
	 * cameras.
	 */
	int ret = cio2->populate();
	if (ret) {
		cerr << "Failed to populate media device " << cio2->deviceNode() << endl;
		return TestFail;
	}

	sensors_ = 0;
	const vector<MediaEntity *> &entities = cio2->entities();
	for (MediaEntity *entity : entities) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR)
			sensors_++;
	}

	enumerator.reset(nullptr);

	cameraManager_ = new CameraManager();
	ret = cameraManager_->start();
	if (ret) {
		cerr << "Failed to start the CameraManager" << endl;
		return TestFail;
	}

	return 0;
}

int IPU3PipelineTest::run()
{
	auto cameras = cameraManager_->cameras();
	for (const std::shared_ptr<Camera> &cam : cameras)
		cout << "Found camera '" << cam->id() << "'" << endl;

	if (cameras.size() != sensors_) {
		cerr << cameras.size() << " cameras registered, but " << sensors_
		     << " were expected" << endl;
		return TestFail;
	}

	return TestPass;
}

void IPU3PipelineTest::cleanup()
{
	cameraManager_->stop();
	delete cameraManager_;
}

TEST_REGISTER(IPU3PipelineTest)
