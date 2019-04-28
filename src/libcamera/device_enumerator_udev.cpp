/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * device_enumerator_udev.cpp - udev-based device enumerator
 */

#include "device_enumerator_udev.h"

#include <fcntl.h>
#include <libudev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <libcamera/event_notifier.h>

#include "log.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(DeviceEnumerator)

DeviceEnumeratorUdev::DeviceEnumeratorUdev()
	: udev_(nullptr)
{
}

DeviceEnumeratorUdev::~DeviceEnumeratorUdev()
{
	delete notifier_;

	if (monitor_)
		udev_monitor_unref(monitor_);
	if (udev_)
		udev_unref(udev_);
}

int DeviceEnumeratorUdev::init()
{
	int ret;

	if (udev_)
		return -EBUSY;

	udev_ = udev_new();
	if (!udev_)
		return -ENODEV;

	monitor_ = udev_monitor_new_from_netlink(udev_, "udev");
	if (!monitor_)
		return -ENODEV;

	ret = udev_monitor_filter_add_match_subsystem_devtype(monitor_, "media",
							      nullptr);
	if (ret < 0)
		return ret;

	return 0;
}

int DeviceEnumeratorUdev::enumerate()
{
	struct udev_enumerate *udev_enum = nullptr;
	struct udev_list_entry *ents, *ent;
	int ret;

	udev_enum = udev_enumerate_new(udev_);
	if (!udev_enum)
		return -ENOMEM;

	ret = udev_enumerate_add_match_subsystem(udev_enum, "media");
	if (ret < 0)
		goto done;

	ret = udev_enumerate_scan_devices(udev_enum);
	if (ret < 0)
		goto done;

	ents = udev_enumerate_get_list_entry(udev_enum);
	if (!ents)
		goto done;

	udev_list_entry_foreach(ent, ents) {
		struct udev_device *dev;
		const char *devnode;
		const char *syspath = udev_list_entry_get_name(ent);

		dev = udev_device_new_from_syspath(udev_, syspath);
		if (!dev) {
			LOG(DeviceEnumerator, Warning)
				<< "Failed to get device for '"
				<< syspath << "', skipping";
			continue;
		}

		devnode = udev_device_get_devnode(dev);
		if (!devnode) {
			udev_device_unref(dev);
			ret = -ENODEV;
			goto done;
		}

		addDevice(devnode);

		udev_device_unref(dev);
	}
done:
	udev_enumerate_unref(udev_enum);
	if (ret < 0)
		return ret;

	ret = udev_monitor_enable_receiving(monitor_);
	if (ret < 0)
		return ret;

	int fd = udev_monitor_get_fd(monitor_);
	notifier_ = new EventNotifier(fd, EventNotifier::Read);
	notifier_->activated.connect(this, &DeviceEnumeratorUdev::udevNotify);

	return 0;
}

std::string DeviceEnumeratorUdev::lookupDeviceNode(int major, int minor)
{
	struct udev_device *device;
	const char *name;
	dev_t devnum;
	std::string deviceNode = std::string();

	devnum = makedev(major, minor);
	device = udev_device_new_from_devnum(udev_, 'c', devnum);
	if (!device)
		return std::string();

	name = udev_device_get_devnode(device);
	if (name)
		deviceNode = name;

	udev_device_unref(device);

	return deviceNode;
}

void DeviceEnumeratorUdev::udevNotify(EventNotifier *notifier)
{
	struct udev_device *dev = udev_monitor_receive_device(monitor_);
	std::string action(udev_device_get_action(dev));
	std::string deviceNode(udev_device_get_devnode(dev));

	LOG(DeviceEnumerator, Debug)
		<< action << " device " << udev_device_get_devnode(dev);

	if (action == "add") {
		addDevice(deviceNode);
	} else if (action == "remove") {
		removeDevice(deviceNode);
	}

	udev_device_unref(dev);
}

} /* namespace libcamera */
