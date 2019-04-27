/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * device_enumerator_udev.h - udev-based device enumerator
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__

#include <string>

#include "device_enumerator.h"

struct udev;
struct udev_monitor;

namespace libcamera {

class EventNotifier;

class DeviceEnumeratorUdev : public DeviceEnumerator
{
public:
	DeviceEnumeratorUdev();
	~DeviceEnumeratorUdev();

	int init() final;
	int enumerate() final;

private:
	struct udev *udev_;
	struct udev_monitor *monitor_;
	EventNotifier *notifier_;

	std::string lookupDeviceNode(int major, int minor) final;

	void udevNotify(EventNotifier *notifier);
};

} /* namespace libcamera */

#endif	/* __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__ */
