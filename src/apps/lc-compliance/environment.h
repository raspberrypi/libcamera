/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Collabora Ltd.
 *
 * environment.h - Common environment for tests
 */

#pragma once

#include <libcamera/libcamera.h>

class Environment
{
public:
	static Environment *get();

	void setup(libcamera::CameraManager *cm, std::string cameraId);

	const std::string &cameraId() const { return cameraId_; }
	libcamera::CameraManager *cm() const { return cm_; }

private:
	Environment() = default;

	std::string cameraId_;
	libcamera::CameraManager *cm_;
};
