/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * libcamera v4l2device test base class
 */

#pragma once

#include <memory>

#include <libcamera/framebuffer.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "test.h"

class V4L2VideoDeviceTest : public Test
{
public:
	V4L2VideoDeviceTest(const char *driver, const char *entity)
		: driver_(driver), entity_(entity), sensor_(nullptr),
		  debayer_(nullptr), capture_(nullptr)
	{
	}

protected:
	int init();
	void cleanup();

	std::string driver_;
	std::string entity_;
	std::unique_ptr<libcamera::DeviceEnumerator> enumerator_;
	std::shared_ptr<libcamera::MediaDevice> media_;
	std::unique_ptr<libcamera::CameraSensor> sensor_;
	libcamera::V4L2Subdevice *debayer_;
	libcamera::V4L2VideoDevice *capture_;
	std::vector<std::unique_ptr<libcamera::FrameBuffer>> buffers_;
};
