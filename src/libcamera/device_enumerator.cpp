/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * device_enumerator.cpp - Enumeration and matching
 */

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

} /* namespace libcamera */
