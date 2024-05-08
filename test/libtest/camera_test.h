/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera camera test base class
 */

#pragma once

#include <memory>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

class CameraTest
{
public:
	CameraTest(const char *name, bool isolate = false);
	~CameraTest();

protected:
	libcamera::CameraManager *cm_;
	std::shared_ptr<libcamera::Camera> camera_;
	int status_;
};
