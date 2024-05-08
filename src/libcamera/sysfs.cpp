/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Miscellaneous utility functions to access sysfs
 */

#include "libcamera/internal/sysfs.h"

#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <sys/sysmacros.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

/**
 * \file sysfs.h
 * \brief Miscellaneous utility functions to access sysfs
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(SysFs)

namespace sysfs {

/**
 * \brief Retrieve the sysfs path for a character device
 * \param[in] deviceNode Path to character device node
 * \return The sysfs path on success or an empty string on failure
 */
std::string charDevPath(const std::string &deviceNode)
{
	struct stat st;
	int ret = stat(deviceNode.c_str(), &st);
	if (ret < 0) {
		ret = -errno;
		LOG(SysFs, Error)
			<< "Unable to stat '" << deviceNode << "': "
			<< strerror(-ret);
		return {};
	}

	std::ostringstream dev("/sys/dev/char/", std::ios_base::ate);
	dev << major(st.st_rdev) << ":" << minor(st.st_rdev);

	return dev.str();
}

/**
 * \brief Retrieve the path of the firmware node for a device
 * \param[in] device Path in sysfs to search
 *
 * Physical devices in a system are described by the system firmware. Depending
 * on the type of platform, devices are identified using different naming
 * schemes. The Linux kernel abstract those differences with "firmware nodes".
 * This function retrieves the firmware node path corresponding to the
 * \a device.
 *
 * For DT-based systems, the path is the full name of the DT node that
 * represents the device. For ACPI-based systems, the path is the absolute
 * namespace path to the ACPI object that represents the device. In both cases,
 * the path is guaranteed to be unique and persistent as long as the system
 * firmware is not modified.
 *
 * \return The firmware node path on success or an empty string on failure
 */
std::string firmwareNodePath(const std::string &device)
{
	std::string fwPath, node;
	struct stat st;

	/* Lookup for DT-based systems */
	node = device + "/of_node";
	if (!stat(node.c_str(), &st)) {
		char *ofPath = realpath(node.c_str(), nullptr);
		if (!ofPath)
			return {};

		static const char prefix[] = "/sys/firmware/devicetree";
		if (strncmp(ofPath, prefix, strlen(prefix)) == 0)
			fwPath = ofPath + strlen(prefix);
		else
			fwPath = ofPath;

		free(ofPath);

		return fwPath;
	}

	/* Lookup for ACPI-based systems */
	node = device + "/firmware_node/path";
	if (File::exists(node)) {
		std::ifstream file(node);
		if (!file.is_open())
			return {};

		std::getline(file, fwPath);
		file.close();

		return fwPath;
	}

	return {};
}

} /* namespace sysfs */

} /* namespace libcamera */
