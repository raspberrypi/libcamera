/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * sysfs.cpp - Miscellaneous utility functions to access sysfs
 */

#include "libcamera/internal/sysfs.h"

#include <sstream>
#include <sys/stat.h>
#include <sys/sysmacros.h>

#include "libcamera/internal/log.h"

/**
 * \file sysfs.h
 * \brief Miscellaneous utility functions to access sysfs
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(SysFs);

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

} /* namespace sysfs */

} /* namespace libcamera */
