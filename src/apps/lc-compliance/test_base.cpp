/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Collabora Ltd.
 *
 * test_base.cpp - Base definitions for tests
 */

#include "test_base.h"

#include "environment.h"

void CameraHolder::acquireCamera()
{
	Environment *env = Environment::get();

	camera_ = env->cm()->get(env->cameraId());

	ASSERT_EQ(camera_->acquire(), 0);
}

void CameraHolder::releaseCamera()
{
	if (!camera_)
		return;

	camera_->release();
	camera_.reset();
}
