/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * vl42device_test.h - libcamera v4l2device test base class
 */
#ifndef __LIBCAMERA_V4L2_DEVICE_TEST_H_
#define __LIBCAMERA_V4L2_DEVICE_TEST_H_

#include <memory>

#include <libcamera/buffer.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "test.h"

using namespace libcamera;

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
	std::unique_ptr<DeviceEnumerator> enumerator_;
	std::shared_ptr<MediaDevice> media_;
	CameraSensor *sensor_;
	V4L2Subdevice *debayer_;
	V4L2VideoDevice *capture_;
	std::vector<std::unique_ptr<FrameBuffer>> buffers_;
};

#endif /* __LIBCAMERA_V4L2_DEVICE_TEST_H_ */
