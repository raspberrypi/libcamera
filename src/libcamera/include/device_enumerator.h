/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * device_enumerator.h - API to enumerate and find media devices
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_H__

#include <map>
#include <string>
#include <vector>

#include <linux/media.h>

namespace libcamera {

class DeviceInfo
{
public:
	DeviceInfo(const std::string &devnode, const struct media_device_info &info,
		   const std::map<std::string, std::string> &entities);

	int acquire();
	void release();
	bool busy() const;

	const std::string &devnode() const;
	const struct media_device_info &info() const;
	std::vector<std::string> entities() const;

	int lookup(const std::string &name, std::string &devnode) const;

private:
	bool acquired_;

	std::string devnode_;
	struct media_device_info info_;
	std::map<std::string, std::string> entities_;
};

} /* namespace libcamera */

#endif	/* __LIBCAMERA_DEVICE_ENUMERATOR_H__ */
