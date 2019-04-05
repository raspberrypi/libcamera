/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * camera_test.h - libcamera camera test base class
 */
#ifndef __LIBCAMERA_CAMERA_TEST_H__
#define __LIBCAMERA_CAMERA_TEST_H__

#include <libcamera/libcamera.h>

#include "test.h"

using namespace libcamera;

class CameraTest : public Test
{
public:
	CameraTest()
		: cm_(nullptr) {}

protected:
	int init();
	void cleanup();

	std::shared_ptr<Camera> camera_;

private:
	CameraManager *cm_;
};

#endif /* __LIBCAMERA_CAMERA_TEST_H__ */
