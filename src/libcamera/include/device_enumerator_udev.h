/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * device_enumerator_udev.h - udev-based device enumerator
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__

#include <list>
#include <map>
#include <memory>
#include <string>
#include <sys/types.h>

#include "device_enumerator.h"

struct udev;
struct udev_device;
struct udev_monitor;

namespace libcamera {

class EventNotifier;
class MediaDevice;
class MediaEntity;

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

	std::map<std::shared_ptr<MediaDevice>, std::list<dev_t>> deps_;
	std::map<dev_t, std::shared_ptr<MediaDevice>> devnumToDevice_;
	std::map<dev_t, MediaEntity *> devnumToEntity_;

	std::list<dev_t> orphans_;

	int addUdevDevice(struct udev_device *dev);
	int populateMediaDevice(const std::shared_ptr<MediaDevice> &media);
	std::string lookupDeviceNode(int major, int minor) final;

	int addV4L2Device(dev_t devnum);
	void udevNotify(EventNotifier *notifier);
};

} /* namespace libcamera */

#endif	/* __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__ */
