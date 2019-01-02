/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * device_enumerator.cpp - Enumeration and matching
 */

#include <fcntl.h>
#include <libudev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "device_enumerator.h"
#include "log.h"
#include "media_device.h"

/**
 * \file device_enumerator.h
 * \brief Enumerating and matching of media devices
 *
 * The purpose of device enumeration and matching is to find media
 * devices in the system and map one or more media devices to a pipeline
 * handler. During enumeration information about each media device is
 * gathered, transformed and stored.
 *
 * The core of the enumeration is DeviceEnumerator which is responsible
 * for all interactions with the operating system and the entry point
 * for other parts of libcamera.
 *
 * The DeviceEnumerator can enumerate all or specific media devices in
 * the system. When a new media device is added the enumerator creates a
 * corresponding MediaDevice instance.
 *
 * The last functionality provided is the ability to search among the
 * enumerate media devices for one matching information known to the
 * searcher. This is done by populating and passing a DeviceMatch object
 * to the DeviceEnumerator.
 *
 * \todo Add sysfs based device enumerator
 * \todo Add support for hot-plug and hot-unplug.
 */

namespace libcamera {

/**
 * \class DeviceMatch
 * \brief Description of a media device search pattern
 *
 * The DeviceMatch class describes a media device using properties from
 * the v4l2 struct media_device_info, entity names in the media graph or
 * other properties which can be used to identify a media device.
 *
 * The description of a media device can then be passed to an enumerator
 * to try and find a matching media device.
 */

/**
 * \brief Construct a media device search pattern
 *
 * \param[in] driver The Linux device driver name who created the media device
 */
DeviceMatch::DeviceMatch(const std::string &driver)
	: driver_(driver)
{
}

/**
 * \brief Add a media entity name to the search pattern
 *
 * \param[in] entity The name of the entity in the media graph
 */
void DeviceMatch::add(const std::string &entity)
{
	entities_.push_back(entity);
}

/**
 * \brief Compare a search pattern with a media device
 * \param[in] device The media device
 *
 * Matching is performed on the Linux device driver name and entity names
 * from the media graph.
 *
 * \return true if the media device matches the search pattern, false otherwise
 */
bool DeviceMatch::match(const MediaDevice *device) const
{
	if (driver_ != device->driver())
		return false;

	for (const std::string &name : entities_) {
		bool found = false;

		for (const MediaEntity *entity : device->entities()) {
			if (name == entity->name()) {
				found = true;
				break;
			}
		}

		if (!found)
			return false;
	}

	return true;
}

/**
 * \class DeviceEnumerator
 * \brief Enumerate, interrogate, store and search media device information
 *
 * The DeviceEnumerator class is responsible for all interactions with
 * the operation system when searching and interrogating media devices.
 *
 * It is possible to automatically search and add all media devices in
 * the system or specify which media devices should be interrogated
 * in order for a specialized application to open as few resources
 * as possible to get hold of a specific camera.
 *
 * Once one or many media devices have been enumerated it is possible
 * to search among them to try and find a matching device using a
 * DeviceMatch object.
 *
 */

/**
 * \brief Create a new device enumerator matching the systems capabilities
 *
 * Create a enumerator based on resource available to the system. Not all
 * different enumerator types are guaranteed to support all features.
 */
DeviceEnumerator *DeviceEnumerator::create()
{
	DeviceEnumerator *enumerator;

	/* TODO: add compile time checks to only try udev enumerator if libudev is available */
	enumerator = new DeviceEnumeratorUdev();
	if (!enumerator->init())
		return enumerator;

	/*
	 * NOTE: Either udev is not available or initialization of it
	 * failed, use/fallback on sysfs enumerator
	 */

	/* TODO: add a sysfs based enumerator */

	return nullptr;
}

DeviceEnumerator::~DeviceEnumerator()
{
	for (MediaDevice *dev : devices_) {
		if (dev->busy())
			LOG(Error) << "Removing media device while still in use";

		delete dev;
	}
}

/**
 * \brief Add a media device to the enumerator
 *
 * \param[in] devnode path to the media device to add
 *
 * Opens the media device and quires its topology and other information.
 *
 * \return 0 on success none zero otherwise
 */
int DeviceEnumerator::addDevice(const std::string &devnode)
{
	MediaDevice *media = new MediaDevice(devnode);

	int ret = media->open();
	if (ret < 0)
		return ret;

	ret = media->populate();
	if (ret < 0) {
		LOG(Info) << "Unable to populate media device " << devnode <<
			  " (" << strerror(-ret) << "), skipping";
		return ret;
	}

	/* Associate entities to device node paths. */
	for (MediaEntity *entity : media->entities()) {
		if (entity->major() == 0 && entity->minor() == 0)
			continue;

		std::string devnode = lookupDevnode(entity->major(), entity->minor());
		if (devnode.empty())
			return -EINVAL;

		ret = entity->setDeviceNode(devnode);
		if (ret)
			return ret;
	}

	devices_.push_back(media);
	media->close();

	return 0;
}

/**
 * \brief Search available media devices for a pattern match
 *
 * \param[in] dm Search pattern
 *
 * Search in the enumerated media devices that are not already in use
 * for a match described in \a dm. If a match is found and the caller
 * intends to use it the caller is responsible to mark the MediaDevice
 * object as in use and to release it when it's done with it.
 *
 * \return pointer to the matching MediaDevice, nullptr if no match is found
 */
MediaDevice *DeviceEnumerator::search(DeviceMatch &dm) const
{
	for (MediaDevice *dev : devices_) {
		if (dev->busy())
			continue;

		if (dm.match(dev))
			return dev;
	}

	return nullptr;
}

/**
 * \class DeviceEnumeratorUdev
 * \brief Udev implementation of device enumeration
 *
 * Implementation of system enumeration functions using libudev.
 */

DeviceEnumeratorUdev::DeviceEnumeratorUdev()
	: udev_(nullptr)
{
}

DeviceEnumeratorUdev::~DeviceEnumeratorUdev()
{
	if (udev_)
		udev_unref(udev_);
}

/**
 * \brief Initialize the enumerator
 *
 * \retval 0 Initialized
 * \retval -EBUSY Busy (already initialized)
 * \retval -ENODEV Failed to talk to udev
 */
int DeviceEnumeratorUdev::init()
{
	if (udev_)
		return -EBUSY;

	udev_ = udev_new();
	if (!udev_)
		return -ENODEV;

	return 0;
}

/**
 * \brief Enumerate all media devices using udev
 *
 * Find, enumerate and add all media devices in the system to the
 * enumerator.
 *
 * \return 0 on success none zero otherwise
 */
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
			LOG(Warning) << "Failed to get device for '" <<
				syspath << "', skipping";
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
	return ret >= 0 ? 0 : ret;
}

/**
 * \brief Lookup device node from device number using udev
 *
 * Translate a device number (major, minor) to a device node path.
 *
 * \return device node path or empty string if lookup fails.
 *
 */
std::string DeviceEnumeratorUdev::lookupDevnode(int major, int minor)
{
	struct udev_device *device;
	const char *name;
	dev_t devnum;
	std::string devnode = std::string();

	devnum = makedev(major, minor);
	device = udev_device_new_from_devnum(udev_, 'c', devnum);
	if (!device)
		return std::string();

	name = udev_device_get_devnode(device);
	if (name)
		devnode = name;

	udev_device_unref(device);

	return devnode;
}

} /* namespace libcamera */
