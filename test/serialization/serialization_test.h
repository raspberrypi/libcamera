/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * serialization_test.h - Base class for serialization tests
 */

#pragma once

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>

#include "camera_test.h"
#include "test.h"

class SerializationTest : public CameraTest, public Test
{
public:
	SerializationTest()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

	static bool equals(const libcamera::ControlInfoMap &lhs,
			   const libcamera::ControlInfoMap &rhs);
	static bool equals(const libcamera::ControlList &lhs,
			   const libcamera::ControlList &rhs);
};
