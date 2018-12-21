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

namespace libcamera {

/* -----------------------------------------------------------------------------
 * DeviceInfo
 */

DeviceInfo::DeviceInfo(const std::string &devnode, const struct media_device_info &info,
		       const std::map<std::string, std::string> &entities)
	: acquired_(false), devnode_(devnode), info_(info), entities_(entities)
{
	for (const auto &entity : entities_)
		LOG(Info) << "Device: " << devnode_ << " Entity: '" << entity.first << "' -> " << entity.second;
}

int DeviceInfo::acquire()
{
	if (acquired_)
		return -EBUSY;

	acquired_ = true;

	return 0;
}

void DeviceInfo::release()
{
	acquired_ = false;
}

bool DeviceInfo::busy() const
{
	return acquired_;
}

const std::string &DeviceInfo::devnode() const
{
	return devnode_;
}

const struct media_device_info &DeviceInfo::info() const
{
	return info_;
}

std::vector<std::string> DeviceInfo::entities() const
{
	std::vector<std::string> entities;

	for (const auto &entity : entities_)
		entities.push_back(entity.first);

	return entities;
}

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

/* -----------------------------------------------------------------------------
 * DeviceMatch
 */

DeviceMatch::DeviceMatch(const std::string &driver)
	: driver_(driver)
{
}

void DeviceMatch::add(const std::string &entity)
{
	entities_.push_back(entity);
}

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

/* -----------------------------------------------------------------------------
 * Enumerator Base
 */

DeviceEnumerator::~DeviceEnumerator()
{
	for (DeviceInfo *dev : devices_) {
		if (dev->busy())
			LOG(Error) << "Removing device info while still in use";

		delete dev;
	}
}

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

} /* namespace libcamera */
