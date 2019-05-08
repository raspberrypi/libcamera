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

#include "test.h"

#include "device_enumerator.h"
#include "media_device.h"
#include "v4l2_device.h"

using namespace libcamera;

class V4L2DeviceTest : public Test
{
public:
	V4L2DeviceTest(const char *driver, const char *entity)
		: driver_(driver), entity_(entity), capture_(nullptr)
	{
	}

protected:
	int init();
	void cleanup();

	std::string driver_;
	std::string entity_;
	std::unique_ptr<DeviceEnumerator> enumerator_;
	std::shared_ptr<MediaDevice> media_;
	V4L2Device *capture_;
	BufferPool pool_;
};

#endif /* __LIBCAMERA_V4L2_DEVICE_TEST_H_ */
