/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * Based on test/pipeline/ipu3/ipu3_pipeline_test.cpp
 *
 * rkisp1_pipeline_test.cpp - Rockchip RK3399 rkisp1 pipeline test
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
 * Verify that the RK3399 pipeline handler gets matched and cameras
 * are enumerated correctly.
 *
 * The test is supposed to be run on rockchip platform.
 *
 * The test lists all cameras registered in the system, if any camera is
 * available at all.
 */
class RKISP1PipelineTest : public Test
{
protected:
	int init();
	int run();
	void cleanup();

private:
	CameraManager *cameraManager_;
	unsigned int sensors_;
};

int RKISP1PipelineTest::init()
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

	DeviceMatch dm("rkisp1");

	std::shared_ptr<MediaDevice> rkisp1 = enumerator->search(dm);
	if (!rkisp1) {
		cerr << "Failed to find rkisp1: test skip" << endl;
		return TestSkip;
	}

	int ret = rkisp1->populate();
	if (ret) {
		cerr << "Failed to populate media device "
		     << rkisp1->deviceNode() << endl;
		return TestFail;
	}

	sensors_ = 0;
	const vector<MediaEntity *> &entities = rkisp1->entities();
	for (MediaEntity *entity : entities) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR)
			sensors_++;
	}

	cameraManager_ = new CameraManager();
	ret = cameraManager_->start();
	if (ret) {
		cerr << "Failed to start the CameraManager" << endl;
		return TestFail;
	}

	return 0;
}

int RKISP1PipelineTest::run()
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

void RKISP1PipelineTest::cleanup()
{
	cameraManager_->stop();
	delete cameraManager_;
}

TEST_REGISTER(RKISP1PipelineTest)
