/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * device_enumerator_sysfs.h - sysfs-based device enumerator
 */
#ifndef __LIBCAMERA_INTERNAL_DEVICE_ENUMERATOR_SYSFS_H__
#define __LIBCAMERA_INTERNAL_DEVICE_ENUMERATOR_SYSFS_H__

#include <memory>
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

#endif /* __LIBCAMERA_INTERNAL_DEVICE_ENUMERATOR_SYSFS_H__ */
