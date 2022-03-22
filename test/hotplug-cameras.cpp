/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Umang Jain <email@uajain.com>
 *
 * hotplug-cameras.cpp - Test cameraAdded/cameraRemoved signals in CameraManager
 */

#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <unistd.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/file.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "test.h"

using namespace libcamera;
using namespace std::chrono_literals;

class HotplugTest : public Test
{
protected:
	void cameraAddedHandler([[maybe_unused]] std::shared_ptr<Camera> cam)
	{
		cameraAdded_ = true;
	}

	void cameraRemovedHandler([[maybe_unused]] std::shared_ptr<Camera> cam)
	{
		cameraRemoved_ = true;
	}

	int init()
	{
		if (!File::exists("/sys/module/uvcvideo")) {
			std::cout << "uvcvideo driver is not loaded, skipping" << std::endl;
			return TestSkip;
		}

		if (geteuid() != 0) {
			std::cout << "This test requires root permissions, skipping" << std::endl;
			return TestSkip;
		}

		cm_ = new CameraManager();
		if (cm_->start()) {
			std::cout << "Failed to start camera manager" << std::endl;
			return TestFail;
		}

		cameraAdded_ = false;
		cameraRemoved_ = false;

		cm_->cameraAdded.connect(this, &HotplugTest::cameraAddedHandler);
		cm_->cameraRemoved.connect(this, &HotplugTest::cameraRemovedHandler);

		return 0;
	}

	int run()
	{
		DIR *dir;
		struct dirent *dirent;
		std::string uvcDeviceDir;

		dir = opendir(uvcDriverDir_.c_str());
		/* Find a UVC device directory, which we can bind/unbind. */
		while ((dirent = readdir(dir)) != nullptr) {
			if (!File::exists(uvcDriverDir_ + dirent->d_name + "/video4linux"))
				continue;

			uvcDeviceDir = dirent->d_name;
			break;
		}
		closedir(dir);

		/* If no UVC device found, skip the test. */
		if (uvcDeviceDir.empty())
			return TestSkip;

		/* Unbind a camera and process events. */
		std::ofstream(uvcDriverDir_ + "unbind", std::ios::binary)
			<< uvcDeviceDir;
		Timer timer;
		timer.start(1000ms);
		while (timer.isRunning() && !cameraRemoved_)
			Thread::current()->eventDispatcher()->processEvents();
		if (!cameraRemoved_) {
			std::cout << "Camera unplug not detected" << std::endl;
			return TestFail;
		}

		/* Bind the camera again and process events. */
		std::ofstream(uvcDriverDir_ + "bind", std::ios::binary)
			<< uvcDeviceDir;
		timer.start(1000ms);
		while (timer.isRunning() && !cameraAdded_)
			Thread::current()->eventDispatcher()->processEvents();
		if (!cameraAdded_) {
			std::cout << "Camera plug not detected" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup()
	{
		cm_->stop();
		delete cm_;
	}

private:
	CameraManager *cm_;
	static const std::string uvcDriverDir_;
	bool cameraRemoved_;
	bool cameraAdded_;
};

const std::string HotplugTest::uvcDriverDir_ = "/sys/bus/usb/drivers/uvcvideo/";

TEST_REGISTER(HotplugTest)
