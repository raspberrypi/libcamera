/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * media_device_test.h - libcamera media device test base class
 */

#pragma once

#include <memory>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"

#include "test.h"

class MediaDeviceTest : public Test
{
public:
	MediaDeviceTest()
		: media_(nullptr), enumerator_(nullptr) {}

protected:
	int init();

	std::shared_ptr<libcamera::MediaDevice> media_;

private:
	std::unique_ptr<libcamera::DeviceEnumerator> enumerator_;
};
