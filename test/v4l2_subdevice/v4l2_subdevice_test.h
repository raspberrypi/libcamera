/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice_test.h - VIMC-based V4L2 subdevice test
 */

#pragma once

#include <libcamera/framebuffer.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "test.h"

class V4L2SubdeviceTest : public Test
{
public:
	V4L2SubdeviceTest()
		: scaler_(nullptr)
	{
	}

protected:
	int init() override;
	void cleanup() override;

	std::unique_ptr<libcamera::DeviceEnumerator> enumerator_;
	std::shared_ptr<libcamera::MediaDevice> media_;
	libcamera::V4L2Subdevice *scaler_;
};
