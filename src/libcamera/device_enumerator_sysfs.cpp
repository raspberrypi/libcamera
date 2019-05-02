/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * device_enumerator_sysfs.cpp - sysfs-based device enumerator
 */

#include "device_enumerator_sysfs.h"

#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "log.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(DeviceEnumerator)

int DeviceEnumeratorSysfs::init()
{
	return 0;
}

int DeviceEnumeratorSysfs::enumerate()
{
	struct dirent *ent;
	DIR *dir;

	static const char * const sysfs_dirs[] = {
		"/sys/subsystem/media/devices",
		"/sys/bus/media/devices",
		"/sys/class/media/devices",
	};

	for (const char *dirname : sysfs_dirs) {
		dir = opendir(dirname);
		if (dir)
			break;
	}

	if (!dir) {
		LOG(DeviceEnumerator, Error)
			<< "No valid sysfs media device directory";
		return -ENODEV;
	}

	while ((ent = readdir(dir)) != nullptr) {
		if (strncmp(ent->d_name, "media", 5))
			continue;

		char *end;
		unsigned int idx = strtoul(ent->d_name + 5, &end, 10);
		if (*end != '\0')
			continue;

		std::string devnode = "/dev/media" + std::to_string(idx);

		/* Verify that the device node exists. */
		struct stat devstat;
		if (stat(devnode.c_str(), &devstat) < 0) {
			LOG(DeviceEnumerator, Warning)
				<< "Device node /dev/media" << idx
				<< " should exist but doesn't";
			continue;
		}

		addDevice(devnode);
	}

	closedir(dir);

	return 0;
}

std::string DeviceEnumeratorSysfs::lookupDeviceNode(int major, int minor)
{
	std::string deviceNode;
	std::string line;
	std::ifstream ueventFile;

	ueventFile.open("/sys/dev/char/" + std::to_string(major) + ":" +
			std::to_string(minor) + "/uevent");
	if (!ueventFile)
		return std::string();

	while (ueventFile >> line) {
		if (line.find("DEVNAME=") == 0) {
			deviceNode = "/dev/" + line.substr(strlen("DEVNAME="));
			break;
		}
	}

	ueventFile.close();

	return deviceNode;
}

} /* namespace libcamera */
