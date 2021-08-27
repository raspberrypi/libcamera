/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * device_enumerator_udev.cpp - udev-based device enumerator
 */

#include "libcamera/internal/device_enumerator_udev.h"

#include <algorithm>
#include <fcntl.h>
#include <libudev.h>
#include <list>
#include <map>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/log.h>

#include "libcamera/internal/media_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(DeviceEnumerator)

DeviceEnumeratorUdev::DeviceEnumeratorUdev()
	: udev_(nullptr), monitor_(nullptr), notifier_(nullptr)
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
		std::unique_ptr<MediaDevice> media =
			createDevice(udev_device_get_devnode(dev));
		if (!media)
			return -ENODEV;

		DependencyMap deps;
		int ret = populateMediaDevice(media.get(), &deps);
		if (ret < 0) {
			LOG(DeviceEnumerator, Warning)
				<< "Failed to populate media device "
				<< media->deviceNode()
				<< " (" << media->driver() << "), skipping";
			return ret;
		}

		if (!deps.empty()) {
			LOG(DeviceEnumerator, Debug)
				<< "Defer media device " << media->deviceNode()
				<< " due to " << deps.size()
				<< " missing dependencies";

			pending_.emplace_back(std::move(media), std::move(deps));
			MediaDeviceDeps *mediaDeps = &pending_.back();
			for (const auto &dep : mediaDeps->deps_)
				devMap_[dep.first] = mediaDeps;

			return 0;
		}

		addDevice(std::move(media));
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
			LOG(DeviceEnumerator, Warning)
				<< "Failed to get device node for '"
				<< syspath << "', skipping";
			continue;
		}

		if (addUdevDevice(dev) < 0)
			LOG(DeviceEnumerator, Warning)
				<< "Failed to add device for '"
				<< syspath << "', skipping";

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

int DeviceEnumeratorUdev::populateMediaDevice(MediaDevice *media, DependencyMap *deps)
{
	std::set<dev_t> children;

	/* Associate entities to device node paths. */
	for (MediaEntity *entity : media->entities()) {
		if (entity->deviceMajor() == 0 && entity->deviceMinor() == 0)
			continue;

		dev_t devnum = makedev(entity->deviceMajor(),
				       entity->deviceMinor());

		/*
		 * If the devnum isn't in the orphans list, add it to the unmet
		 * dependencies.
		 */
		if (orphans_.find(devnum) == orphans_.end()) {
			(*deps)[devnum].push_back(entity);
			continue;
		}

		/*
		 * Otherwise take it from the orphans list. Don't remove the
		 * entry from the list yet as other entities in this media
		 * device may need the same device.
		 */
		std::string deviceNode = lookupDeviceNode(devnum);
		if (deviceNode.empty())
			return -EINVAL;

		int ret = entity->setDeviceNode(deviceNode);
		if (ret)
			return ret;

		children.insert(devnum);
	}

	/* Remove all found children from the orphans list. */
	for (auto it = orphans_.begin(), last = orphans_.end(); it != last;) {
		if (children.find(*it) != children.end())
			it = orphans_.erase(it);
		else
			++it;
	}

	return 0;
}

/**
 * \brief Lookup device node path from device number
 * \param[in] devnum The device number
 *
 * Translate a device number given as \a devnum to a device node path.
 *
 * \return The device node path on success, or an empty string if the lookup
 * fails
 */
std::string DeviceEnumeratorUdev::lookupDeviceNode(dev_t devnum)
{
	struct udev_device *device;
	const char *name;
	std::string deviceNode = std::string();

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
	/*
	 * If the devnum doesn't belong to any media device, add it to the
	 * orphans list.
	 */
	auto it = devMap_.find(devnum);
	if (it == devMap_.end()) {
		orphans_.insert(devnum);
		return 0;
	}

	/*
	 * Set the device node for all entities matching the devnum. Multiple
	 * entities can share the same device node, for instance for V4L2 M2M
	 * devices.
	 */
	std::string deviceNode = lookupDeviceNode(devnum);
	if (deviceNode.empty())
		return -EINVAL;

	MediaDeviceDeps *deps = it->second;
	for (MediaEntity *entity : deps->deps_[devnum]) {
		int ret = entity->setDeviceNode(deviceNode);
		if (ret)
			return ret;
	}

	/*
	 * Remove the devnum from the unmet dependencies for this media device.
	 * If no more dependency is unmet, add the media device to the
	 * enumerator.
	 */
	deps->deps_.erase(devnum);

	if (deps->deps_.empty()) {
		LOG(DeviceEnumerator, Debug)
			<< "All dependencies for media device "
			<< deps->media_->deviceNode() << " found";
		addDevice(std::move(deps->media_));
		pending_.remove(*deps);
	}

	return 0;
}

void DeviceEnumeratorUdev::udevNotify()
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
