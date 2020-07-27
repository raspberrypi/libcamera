/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * serialization_test.h - Base class for serialization tests
 */
#ifndef __LIBCAMERA_SERIALIZATION_TEST_H__
#define __LIBCAMERA_SERIALIZATION_TEST_H__

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>

#include "camera_test.h"
#include "test.h"

using namespace libcamera;

class SerializationTest : public CameraTest, public Test
{
public:
	SerializationTest()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

	static bool equals(const ControlInfoMap &lhs,
			   const ControlInfoMap &rhs);
	static bool equals(const ControlList &lhs,
			   const ControlList &rhs);
};

#endif /* __LIBCAMERA_SERIALIZATION_TEST_H__ */
