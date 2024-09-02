/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * sysfs-based device enumerator
 */

#pragma once

#include <string>

#include "libcamera/internal/device_enumerator.h"

class MediaDevice;

namespace libcamera {

class DeviceEnumeratorSysfs final : public DeviceEnumerator
{
public:
	int init();
	int enumerate();

private:
	int populateMediaDevice(MediaDevice *media);
	std::string lookupDeviceNode(int major, int minor);
};

} /* namespace libcamera */
