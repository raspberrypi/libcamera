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
 * the system. When a new media device is added the enumerator gathers
 * information about it and stores it in a DeviceInfo object.
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
 * \class DeviceInfo
 * \brief Container of information for enumerated device
 *
 * The DeviceInfo class holds information about a media device. It provides
 * methods to retrieve the information stored and to lookup entity names
 * to device node paths. Furthermore it provides a scheme where a device
 * can be acquired and released to indicate if the device is in use.
 *
 * \todo Look into the possibility to replace this with a more complete MediaDevice model.
 */

/**
 * \brief Construct a container of device information
 *
 * \param[in] devnode The path to the device node of the media device
 * \param[in] info Information retrieved from MEDIA_IOC_DEVICE_INFO IOCTL
 * \param[in] entities A map of media graph 'Entity name' -> 'devnode path'
 *
 * The caller is responsible to provide all information for the device.
 */
DeviceInfo::DeviceInfo(const std::string &devnode, const struct media_device_info &info,
		       const std::map<std::string, std::string> &entities)
	: acquired_(false), devnode_(devnode), info_(info), entities_(entities)
{
	for (const auto &entity : entities_)
		LOG(Info) << "Device: " << devnode_ << " Entity: '" << entity.first << "' -> " << entity.second;
}

/**
 * \brief Claim a device for exclusive use
 *
 * Once a device is successfully acquired the caller is responsible to
 * release it once it is done wit it.
 *
 * \retval 0 Device claimed
 * \retval -EBUSY Device already claimed by someone else
 */
int DeviceInfo::acquire()
{
	if (acquired_)
		return -EBUSY;

	acquired_ = true;

	return 0;
}

/**
 * \brief Release a device from exclusive use
 */
void DeviceInfo::release()
{
	acquired_ = false;
}

/**
 * \brief Check if a device is in use
 *
 * \retval true Device is in use
 * \retval false Device is free
 */
bool DeviceInfo::busy() const
{
	return acquired_;
}

/**
 * \brief Retrieve the devnode to the media device
 *
 * \return Path to the media device (example /dev/media0)
 */
const std::string &DeviceInfo::devnode() const
{
	return devnode_;
}

/**
 * \brief Retrieve the media device v4l2 information
 *
 * \return v4l2 specific information structure
 */
const struct media_device_info &DeviceInfo::info() const
{
	return info_;
}

/**
 * \brief List all entities of the device
 *
 * List all media entities names from the media graph which are known
 * and to which this instance can lookup the device node path.
 *
 * \return List of strings
 */
std::vector<std::string> DeviceInfo::entities() const
{
	std::vector<std::string> entities;

	for (const auto &entity : entities_)
		entities.push_back(entity.first);

	return entities;
}

/**
 * \brief Lookup a media entity name and retrieve its device node path
 *
 * \param[in] name Entity name to lookup
 * \param[out] devnode Path to \a name devnode if lookup is successful
 *
 * The caller is responsible to check the return code of the function
 * to determine if the entity name could be looked up.
 *
 * \return 0 on success none zero otherwise
 */
int DeviceInfo::lookup(const std::string &name, std::string &devnode) const
{
	auto it = entities_.find(name);

	if (it == entities_.end()) {
		LOG(Error) << "Trying to lookup entity '" << name << "' which does not exist";
		return -ENODEV;
	}

	devnode = it->second;
	return 0;
}

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
 *
 * \param[in] info Information about a enumerated media device
 *
 * Matching is performed on the Linux device driver name and entity names
 * from the media graph.
 *
 * \retval true The device described in \a info matches search pattern
 * \retval false The device described in \a info do not match search pattern
 */
bool DeviceMatch::match(const DeviceInfo *info) const
{
	if (driver_ != info->info().driver)
		return false;

	for (const std::string &name : entities_) {
		bool found = false;

		for (const std::string &entity : info->entities()) {
			if (name == entity) {
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
	for (DeviceInfo *dev : devices_) {
		if (dev->busy())
			LOG(Error) << "Removing device info while still in use";

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
	int fd, ret;

	struct media_device_info info = {};
	std::map<std::string, std::string> entities;

	fd = open(devnode.c_str(), O_RDWR);
	if (fd < 0) {
		ret = -errno;
		LOG(Info) << "Unable to open " << devnode <<
			  " (" << strerror(-ret) << "), skipping";
		return ret;
	}

	ret = readInfo(fd, info);
	if (ret)
		goto out;

	ret = readTopology(fd, entities);
	if (ret)
		goto out;

	devices_.push_back(new DeviceInfo(devnode, info, entities));
out:
	close(fd);

	return ret;
}

/**
 * \brief Fetch the MEDIA_IOC_DEVICE_INFO from media device
 *
 * \param[in] fd File pointer to media device
 * \param[out] info Information retrieved from MEDIA_IOC_DEVICE_INFO IOCTL
 *
 * Opens the media device and quires its information.
 *
 * \return 0 on success none zero otherwise
 */
int DeviceEnumerator::readInfo(int fd, struct media_device_info &info)
{
	int ret;

	ret = ioctl(fd, MEDIA_IOC_DEVICE_INFO, &info);
	if (ret < 0) {
		ret = -errno;
		LOG(Info) << "Unable to read device info " <<
			  " (" << strerror(-ret) << "), skipping";
		return ret;
	}

	return 0;
}

/**
 * \brief Fetch the topology from media device
 *
 * \param[in] fd File pointer to media device
 * \param[out] entities Map of entity names to device node paths
 *
 * The media graph is retrieved using MEDIA_IOC_G_TOPOLOGY and the
 * result is transformed to a map where the entity name is the key
 * and the filesystem path for that entity device node is the value.
 *
 * \return 0 on success none zero otherwise
 */
int DeviceEnumerator::readTopology(int fd, std::map<std::string, std::string> &entities)
{
	struct media_v2_topology topology;
	struct media_v2_entity *ents = nullptr;
	struct media_v2_interface *ifaces = nullptr;
	struct media_v2_link *links = nullptr;
	int ret;

	while (true) {
		topology = {};

		ret = ioctl(fd, MEDIA_IOC_G_TOPOLOGY, &topology);
		if (ret < 0)
			return -errno;

		__u64 version = topology.topology_version;

		ents = new media_v2_entity[topology.num_entities]();
		ifaces = new media_v2_interface[topology.num_interfaces]();
		links = new media_v2_link[topology.num_links]();
		topology.ptr_entities = reinterpret_cast<__u64>(ents);
		topology.ptr_interfaces = reinterpret_cast<__u64>(ifaces);
		topology.ptr_links = reinterpret_cast<__u64>(links);

		ret = ioctl(fd, MEDIA_IOC_G_TOPOLOGY, &topology);
		if (ret < 0) {
			ret = -errno;
			goto done;
		}

		if (version == topology.topology_version)
			break;

		delete[] links;
		delete[] ifaces;
		delete[] ents;
	}

	for (unsigned int link_id = 0; link_id < topology.num_links; link_id++) {
		unsigned int iface_id, ent_id;
		std::string devnode;

		if ((links[link_id].flags & MEDIA_LNK_FL_LINK_TYPE) !=
		    MEDIA_LNK_FL_INTERFACE_LINK)
			continue;

		for (iface_id = 0; iface_id < topology.num_interfaces; iface_id++)
			if (links[link_id].source_id == ifaces[iface_id].id)
				break;

		for (ent_id = 0; ent_id < topology.num_entities; ent_id++)
			if (links[link_id].sink_id == ents[ent_id].id)
				break;

		if (ent_id >= topology.num_entities ||
		    iface_id >= topology.num_interfaces)
			continue;

		devnode = lookupDevnode(ifaces[iface_id].devnode.major,
					ifaces[iface_id].devnode.minor);
		if (devnode == "")
			break;

		entities[ents[ent_id].name] = devnode;
	}
done:
	delete[] links;
	delete[] ifaces;
	delete[] ents;

	return ret;
}

/**
 * \brief Search available media devices for a pattern match
 *
 * \param[in] dm search pattern
 *
 * Search the enumerated media devices who are not already in use
 * for a match described in \a dm. If a match is found and the caller
 * intends to use it the caller is responsible to mark the DeviceInfo
 * object as in use and to release it when it's done with it.
 *
 * \return pointer to the matching DeviceInfo, nullptr if no match is found
 */
DeviceInfo *DeviceEnumerator::search(DeviceMatch &dm) const
{
	DeviceInfo *info = nullptr;

	for (DeviceInfo *dev : devices_) {
		if (dev->busy())
			continue;

		if (dm.match(dev)) {
			info = dev;
			break;
		}
	}

	return info;
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
			LOG(Error) << "Failed to get device for '" <<
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
