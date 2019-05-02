/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * device_enumerator_sysfs.h - sysfs-based device enumerator
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_SYSFS_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_SYSFS_H__

#include <string>

#include "device_enumerator.h"

namespace libcamera {

class DeviceEnumeratorSysfs final : public DeviceEnumerator
{
public:
	int init();
	int enumerate();

private:
	std::string lookupDeviceNode(int major, int minor);
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_DEVICE_ENUMERATOR_SYSFS_H__ */
