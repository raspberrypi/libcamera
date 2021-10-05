/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Collabora Ltd.
 *
 * environment.cpp - Common environment for tests
 */

#include "environment.h"

using namespace libcamera;

Environment *Environment::get()
{
	static Environment instance;
	return &instance;
}

void Environment::setup(CameraManager *cm, std::string cameraId)
{
	cm_ = cm;
	cameraId_ = cameraId;
}
