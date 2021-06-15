/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * source_paths.cpp - Identify libcamera source and build paths
 */

#include "libcamera/internal/source_paths.h"

#include <dlfcn.h>
#include <elf.h>
#include <link.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <libcamera/base/utils.h>

/**
 * \file source_paths.h
 * \brief Identify the build and source path of a not-yet-installed library
 */

/* musl doesn't declare _DYNAMIC in link.h, declare it manually. */
extern ElfW(Dyn) _DYNAMIC[];

namespace libcamera {

namespace {

/**
 * \brief Check if libcamera is installed or not
 *
 * Utilise the build_rpath dynamic tag which is stripped out by meson at
 * install time to determine at runtime if the library currently executing
 * has been installed or not.
 *
 * \return True if libcamera is installed, false otherwise
 */
bool isLibcameraInstalled()
{
	/*
	 * DT_RUNPATH (DT_RPATH when the linker uses old dtags) is removed on
	 * install.
	 */
	for (const ElfW(Dyn) *dyn = _DYNAMIC; dyn->d_tag != DT_NULL; ++dyn) {
		if (dyn->d_tag == DT_RUNPATH || dyn->d_tag == DT_RPATH)
			return false;
	}

	return true;
}

} /* namespace */

namespace utils {

/**
 * \brief Retrieve the path to the build directory
 *
 * During development, it is useful to run libcamera binaries directly from the
 * build directory without installing them. This function helps components that
 * need to locate resources in the build tree, such as IPA modules or IPA proxy
 * workers, by providing them with the path to the root of the build directory.
 * Callers can then use it to complement or override searches in system-wide
 * directories.
 *
 * If libcamera has been installed, the build directory path is not available
 * and this function returns an empty string.
 *
 * \return The path to the build directory if running from a build, or an empty
 * string otherwise
 */
std::string libcameraBuildPath()
{
	if (isLibcameraInstalled())
		return std::string();

	Dl_info info;

	/* Look up our own symbol. */
	int ret = dladdr(reinterpret_cast<void *>(libcameraBuildPath), &info);
	if (ret == 0)
		return std::string();

	std::string path = dirname(info.dli_fname) + "/../../";

	char *real = realpath(path.c_str(), nullptr);
	if (!real)
		return std::string();

	path = real;
	free(real);

	return path + "/";
}

/**
 * \brief Retrieve the path to the source directory
 *
 * During development, it is useful to run libcamera binaries directly from the
 * build directory without installing them. This function helps components that
 * need to locate resources in the source tree, such as IPA configuration
 * files, by providing them with the path to the root of the source directory.
 * Callers can then use it to complement or override searches in system-wide
 * directories.
 *
 * If libcamera has been installed, the source directory path is not available
 * and this function returns an empty string.
 *
 * \return The path to the source directory if running from a build directory,
 * or an empty string otherwise
 */
std::string libcameraSourcePath()
{
	std::string path = libcameraBuildPath();
	if (path.empty())
		return std::string();

	path += "source";

	char *real = realpath(path.c_str(), nullptr);
	if (!real)
		return std::string();

	path = real;
	free(real);

	struct stat statbuf;
	int ret = stat(path.c_str(), &statbuf);
	if (ret < 0 || (statbuf.st_mode & S_IFMT) != S_IFDIR)
		return std::string();

	return path + "/";
}

} /* namespace utils */

} /* namespace libcamera */
