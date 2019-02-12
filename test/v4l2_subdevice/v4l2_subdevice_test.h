/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice_test.h - VIMC-based V4L2 subdevice test
 */

#ifndef __LIBCAMERA_V4L2_SUBDEVICE_TEST_H__
#define __LIBCAMERA_V4L2_SUBDEVICE_TEST_H__

#include <libcamera/buffer.h>

#include "device_enumerator.h"
#include "media_device.h"
#include "test.h"
#include "v4l2_subdevice.h"

using namespace libcamera;

class V4L2SubdeviceTest : public Test
{
public:
	V4L2SubdeviceTest()
		: scaler_(nullptr){};

protected:
	int init() override;
	void cleanup() override;

	std::unique_ptr<DeviceEnumerator> enumerator_;
	std::shared_ptr<MediaDevice> media_;
	V4L2Subdevice *scaler_;
};

#endif /* __LIBCAMERA_V4L2_SUBDEVICE_TEST_H__ */
