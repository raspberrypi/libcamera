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
#include "utils.h"

/**
 * \file device_enumerator.h
 * \brief Enumeration and matching of media devices
 *
 * The purpose of device enumeration and matching is to find media devices in
 * the system and map them to pipeline handlers.
 *
 * At the core of the enumeration is the DeviceEnumerator class, responsible
 * for enumerating all media devices in the system. It handles all interactions
 * with the operating system in a platform-specific way. For each media device
 * found an instance of MediaDevice is created to store information about the
 * device gathered from the kernel through the Media Controller API.
 *
 * The DeviceEnumerator can enumerate all or specific media devices in the
 * system. When a new media device is added the enumerator creates a
 * corresponding MediaDevice instance.
 *
 * The enumerator supports searching among enumerated devices based on criteria
 * expressed in a DeviceMatch object.
 *
 * \todo Add sysfs based device enumerator.
 * \todo Add support for hot-plug and hot-unplug.
 */

namespace libcamera {

/**
 * \class DeviceMatch
 * \brief Description of a media device search pattern
 *
 * The DeviceMatch class describes a media device using properties from the
 * Media Controller struct media_device_info, entity names in the media graph
 * or other properties that can be used to identify a media device.
 *
 * The description is meant to be filled by pipeline managers and passed to a
 * device enumerator to find matching media devices.
 */

/**
 * \brief Construct a media device search pattern
 * \param[in] driver The Linux device driver name that created the media device
 */
DeviceMatch::DeviceMatch(const std::string &driver)
	: driver_(driver)
{
}

/**
 * \brief Add a media entity name to the search pattern
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
 * Matching is performed on the Linux device driver name and entity names from
 * the media graph. A match is found if both the driver name matches and the
 * media device contains all the entities listed in the search pattern.
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
 * \brief Enumerate, store and search media devices
 *
 * The DeviceEnumerator class is responsible for all interactions with the
 * operating system related to media devices. It enumerates all media devices
 * in the system, and for each device found creates an instance of the
 * MediaDevice class and stores it internally. The list of media devices can
 * then be searched using DeviceMatch search patterns.
 *
 * The enumerator also associates media device entities with device node paths.
 */

/**
 * \brief Create a new device enumerator matching the systems capabilities
 *
 * Depending on how the operating system handles device detection, hot-plug
 * notification and device node lookup, different device enumerator
 * implementations may be needed. This function creates the best enumerator for
 * the operating system based on the available resources. Not all different
 * enumerator types are guaranteed to support all features.
 *
 * \return A pointer to the newly created device enumerator on success, or
 * nullptr if an error occurs
 */
std::unique_ptr<DeviceEnumerator> DeviceEnumerator::create()
{
	std::unique_ptr<DeviceEnumerator> enumerator;

	/**
	 * \todo Add compile time checks to only try udev enumerator if libudev
	 * is available.
	 */
	enumerator = utils::make_unique<DeviceEnumeratorUdev>();
	if (!enumerator->init())
		return enumerator;

	/*
	 * Either udev is not available or udev initialization failed. Fall back
	 * on the sysfs enumerator.
	 */

	/** \todo Add a sysfs-based enumerator. */

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
 * \fn DeviceEnumerator::init()
 * \brief Initialize the enumerator
 * \return 0 on success, or a negative error code otherwise
 * \retval -EBUSY the enumerator has already been initialized
 * \retval -ENODEV the enumerator can't enumerate devices
 */

/**
 * \fn DeviceEnumerator::enumerate()
 * \brief Enumerate all media devices in the system
 *
 * This function finds and add all media devices in the system to the
 * enumerator. It shall be implemented by all subclasses of DeviceEnumerator
 * using system-specific methods.
 *
 * Individual media devices that can't be properly enumerated shall be skipped
 * with a warning message logged, without returning an error. Only errors that
 * prevent enumeration altogether shall be fatal.
 *
 * \return 0 on success, or a negative error code on fatal errors.
 */

/**
 * \brief Add a media device to the enumerator
 * \param[in] devnode path to the media device to add
 *
 * Create a media device for the \a devnode, open it, populate its media graph,
 * and look up device nodes associated with all entities. Store the media device
 * in the internal list for later matching with pipeline handlers.
 *
 * \return 0 on success, or a negative error code if the media device can't be
 * created or populated
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

	LOG(Debug) << "New media device \"" << media->driver()
		   << "\" created from " << devnode;

	/* Associate entities to device node paths. */
	for (MediaEntity *entity : media->entities()) {
		if (entity->deviceMajor() == 0 && entity->deviceMinor() == 0)
			continue;

		std::string devnode = lookupDevnode(entity->deviceMajor(), entity->deviceMinor());
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
 * \param[in] dm Search pattern
 *
 * Search in the enumerated media devices that are not already in use for a
 * match described in \a dm. If a match is found and the caller intends to use
 * it the caller is responsible for acquiring the MediaDevice object and
 * releasing it when done with it.
 *
 * \return pointer to the matching MediaDevice, or nullptr if no match is found
 */
MediaDevice *DeviceEnumerator::search(const DeviceMatch &dm)
{
	for (MediaDevice *dev : devices_) {
		if (dev->busy())
			continue;

		if (dm.match(dev)) {
			LOG(Debug) << "Successful match for media device \""
				   << dev->driver() << "\"";
			return dev;
		}
	}

	return nullptr;
}

/**
 * \fn DeviceEnumerator::lookupDevnode(int major, int minor)
 * \brief Lookup device node path from device number
 * \param major The device major number
 * \param minor The device minor number
 *
 * Translate a device number given as \a major and \a minor to a device node
 * path.
 *
 * \return the device node path on success, or an empty string if the lookup
 * fails
 */

/**
 * \class DeviceEnumeratorUdev
 * \brief Device enumerator based on libudev
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

int DeviceEnumeratorUdev::init()
{
	if (udev_)
		return -EBUSY;

	udev_ = udev_new();
	if (!udev_)
		return -ENODEV;

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
