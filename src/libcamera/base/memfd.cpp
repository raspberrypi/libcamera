/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Anonymous file creation
 */

#include <libcamera/base/memfd.h>

#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <libcamera/base/log.h>

/**
 * \file base/memfd.h
 * \brief Anonymous file creation
 */

#ifndef __DOXYGEN__
namespace {

/* uClibc doesn't provide the file sealing API. */
#if not HAVE_FILE_SEALS
#define F_ADD_SEALS		1033
#define F_SEAL_SHRINK		0x0002
#define F_SEAL_GROW		0x0004
#endif

#if not HAVE_MEMFD_CREATE
int memfd_create(const char *name, unsigned int flags)
{
	return syscall(SYS_memfd_create, name, flags);
}
#endif

} /* namespace */
#endif /* __DOXYGEN__ */

namespace libcamera {

LOG_DECLARE_CATEGORY(File)

/**
 * \class MemFd
 * \brief Helper class to create anonymous files
 *
 * Anonymous files behave like regular files, and can be modified, truncated,
 * memory-mapped and so on. Unlike regular files, they however live in RAM and
 * don't have permanent backing storage.
 */

/**
 * \enum MemFd::Seal
 * \brief Seals for the MemFd::create() function
 * \var MemFd::Seal::None
 * \brief No seals (used as default value)
 * \var MemFd::Seal::Shrink
 * \brief Prevent the memfd from shrinking
 * \var MemFd::Seal::Grow
 * \brief Prevent the memfd from growing
 */

/**
 * \typedef MemFd::Seals
 * \brief A bitwise combination of MemFd::Seal values
 */

/**
 * \brief Create an anonymous file
 * \param[in] name The file name (displayed in symbolic links in /proc/self/fd/)
 * \param[in] size The file size
 * \param[in] seals The file seals
 *
 * This function is a helper that wraps anonymous file (memfd) creation and
 * sets the file size and optional seals.
 *
 * \return The descriptor of the anonymous file if creation succeeded, or an
 * invalid UniqueFD otherwise
 */
UniqueFD MemFd::create(const char *name, std::size_t size, Seals seals)
{
	int ret = memfd_create(name, MFD_ALLOW_SEALING | MFD_CLOEXEC);
	if (ret < 0) {
		ret = errno;
		LOG(File, Error)
			<< "Failed to allocate memfd storage for " << name
			<< ": " << strerror(ret);
		return {};
	}

	UniqueFD memfd(ret);

	ret = ftruncate(memfd.get(), size);
	if (ret < 0) {
		ret = errno;
		LOG(File, Error)
			<< "Failed to set memfd size for " << name
			<< ": " << strerror(ret);
		return {};
	}

	if (seals) {
		int fileSeals = (seals & Seal::Shrink ? F_SEAL_SHRINK : 0)
			      | (seals & Seal::Grow ? F_SEAL_GROW : 0);

		ret = fcntl(memfd.get(), F_ADD_SEALS, fileSeals);
		if (ret < 0) {
			ret = errno;
			LOG(File, Error)
				<< "Failed to seal the memfd for " << name
				<< ": " << strerror(ret);
			return {};
		}
	}

	return memfd;
}

} /* namespace libcamera */
