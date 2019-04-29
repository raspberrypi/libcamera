/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * media_device_test.h - libcamera media device test base class
 */
#ifndef __LIBCAMERA_MEDIADEVICE_TEST_H__
#define __LIBCAMERA_MEDIADEVICE_TEST_H__

#include <memory>

#include "device_enumerator.h"
#include "media_device.h"

#include "test.h"

using namespace libcamera;

class MediaDeviceTest : public Test
{
public:
	MediaDeviceTest()
		: media_(nullptr), enumerator_(nullptr) {}

protected:
	int init();

	std::shared_ptr<MediaDevice> media_;

private:
	std::unique_ptr<DeviceEnumerator> enumerator_;
};

#endif /* __LIBCAMERA_MEDIADEVICE_TEST_H__ */
