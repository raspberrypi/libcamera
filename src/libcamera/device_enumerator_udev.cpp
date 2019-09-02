/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * device_enumerator_udev.cpp - udev-based device enumerator
 */

#include "device_enumerator_udev.h"

#include <algorithm>
#include <list>
#include <map>

#include <fcntl.h>
#include <libudev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <libcamera/event_notifier.h>

#include "log.h"
#include "media_device.h"

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

	ret = udev_monitor_filter_add_match_subsystem_devtype(monitor_, "video4linux",
							      nullptr);
	if (ret < 0)
		return ret;

	return 0;
}

int DeviceEnumeratorUdev::addUdevDevice(struct udev_device *dev)
{
	const char *subsystem = udev_device_get_subsystem(dev);
	if (!subsystem)
		return -ENODEV;

	if (!strcmp(subsystem, "media")) {
		std::shared_ptr<MediaDevice> media =
			createDevice(udev_device_get_devnode(dev));
		if (!media)
			return -ENODEV;

		int ret = populateMediaDevice(media);
		if (ret == 0)
			addDevice(media);
		return 0;
	}

	if (!strcmp(subsystem, "video4linux")) {
		addV4L2Device(udev_device_get_devnum(dev));
		return 0;
	}

	return -ENODEV;
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

	ret = udev_enumerate_add_match_subsystem(udev_enum, "video4linux");
	if (ret < 0)
		goto done;

	ret = udev_enumerate_add_match_is_initialized(udev_enum);
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

		ret = addUdevDevice(dev);
		udev_device_unref(dev);
		if (ret < 0)
			break;
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

int DeviceEnumeratorUdev::populateMediaDevice(const std::shared_ptr<MediaDevice> &media)
{
	unsigned int pendingNodes = 0;
	int ret;

	/* Associate entities to device node paths. */
	for (MediaEntity *entity : media->entities()) {
		if (entity->deviceMajor() == 0 && entity->deviceMinor() == 0)
			continue;

		std::string deviceNode = lookupDeviceNode(entity->deviceMajor(),
							  entity->deviceMinor());
		dev_t devnum = makedev(entity->deviceMajor(),
				       entity->deviceMinor());

		/* Take device from orphan list first, if it is in the list. */
		if (std::find(orphans_.begin(), orphans_.end(), devnum) != orphans_.end()) {
			if (deviceNode.empty())
				return -EINVAL;

			ret = entity->setDeviceNode(deviceNode);
			if (ret)
				return ret;

			orphans_.remove(devnum);
			continue;
		}

		deps_[media].push_back(devnum);
		devnumToDevice_[devnum] = media;
		devnumToEntity_[devnum] = entity;
		pendingNodes++;
	}

	return pendingNodes;
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

/**
 * \brief Add a V4L2 device to the media device that it belongs to
 * \param[in] devnum major:minor number of V4L2 device to add, as a dev_t
 *
 * Add V4L2 device identified by \a devnum to the MediaDevice that it belongs
 * to, if such a MediaDevice has been created. Otherwise add the V4L2 device
 * to the orphan list. If the V4L2 device is added to a MediaDevice, and it is
 * the last V4L2 device that the MediaDevice needs, then the MediaDevice is
 * added to the DeviceEnumerator, where it is available for pipeline handlers.
 *
 * \return 0 on success or a negative error code otherwise
 */
int DeviceEnumeratorUdev::addV4L2Device(dev_t devnum)
{
	MediaEntity *entity = devnumToEntity_[devnum];
	if (!entity) {
		orphans_.push_back(devnum);
		return 0;
	}

	std::string deviceNode = lookupDeviceNode(entity->deviceMajor(),
						  entity->deviceMinor());
	if (deviceNode.empty())
		return -EINVAL;

	int ret = entity->setDeviceNode(deviceNode);
	if (ret)
		return ret;

	std::shared_ptr<MediaDevice> media = devnumToDevice_[devnum];
	deps_[media].remove(devnum);
	devnumToDevice_.erase(devnum);
	devnumToEntity_.erase(devnum);

	if (deps_[media].empty()) {
		addDevice(media);
		deps_.erase(media);
	}

	return 0;
}

void DeviceEnumeratorUdev::udevNotify(EventNotifier *notifier)
{
	struct udev_device *dev = udev_monitor_receive_device(monitor_);
	std::string action(udev_device_get_action(dev));
	std::string deviceNode(udev_device_get_devnode(dev));

	LOG(DeviceEnumerator, Debug)
		<< action << " device " << udev_device_get_devnode(dev);

	if (action == "add") {
		addUdevDevice(dev);
	} else if (action == "remove") {
		const char *subsystem = udev_device_get_subsystem(dev);
		if (subsystem && !strcmp(subsystem, "media"))
			removeDevice(deviceNode);
	}

	udev_device_unref(dev);
}

} /* namespace libcamera */
