/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * Helper class for dma-buf allocations.
 */

#include "libcamera/internal/dma_buf_allocator.h"

#include <array>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/udmabuf.h>

#include <libcamera/base/log.h>

/**
 * \file dma_buf_allocator.cpp
 * \brief dma-buf allocator
 */

namespace libcamera {

#ifndef __DOXYGEN__
struct DmaBufAllocatorInfo {
	DmaBufAllocator::DmaBufAllocatorFlag type;
	const char *deviceNodeName;
};
#endif

static constexpr std::array<DmaBufAllocatorInfo, 4> providerInfos = { {
	/*
	 * /dev/dma_heap/linux,cma is the CMA dma-heap. When the cma heap size is
	 * specified on the kernel command line, this gets renamed to "reserved".
	 */
	{ DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap, "/dev/dma_heap/linux,cma" },
	{ DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap, "/dev/dma_heap/reserved" },
	{ DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap, "/dev/dma_heap/system" },
	{ DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf, "/dev/udmabuf" },
} };

LOG_DEFINE_CATEGORY(DmaBufAllocator)

/**
 * \class DmaBufAllocator
 * \brief Helper class for dma-buf allocations
 *
 * This class wraps a userspace dma-buf provider selected at construction time,
 * and exposes functions to allocate dma-buffers from this provider.
 *
 * Different providers may provide dma-buffers with different properties for
 * the underlying memory. Which providers are acceptable is specified through
 * the type argument passed to the DmaBufAllocator() constructor.
 */

/**
 * \enum DmaBufAllocator::DmaBufAllocatorFlag
 * \brief Type of the dma-buf provider
 * \var DmaBufAllocator::CmaHeap
 * \brief Allocate from a CMA dma-heap, providing physically-contiguous memory
 * \var DmaBufAllocator::SystemHeap
 * \brief Allocate from the system dma-heap, using the page allocator
 * \var DmaBufAllocator::UDmaBuf
 * \brief Allocate using a memfd + /dev/udmabuf
 */

/**
 * \typedef DmaBufAllocator::DmaBufAllocatorFlags
 * \brief A bitwise combination of DmaBufAllocator::DmaBufAllocatorFlag values
 */

/**
 * \brief Construct a DmaBufAllocator of a given type
 * \param[in] type The type(s) of the dma-buf providers to allocate from
 *
 * The dma-buf provider type is selected with the \a type parameter, which
 * defaults to the CMA heap. If no provider of the given type can be accessed,
 * the constructed DmaBufAllocator instance is invalid as indicated by
 * the isValid() function.
 *
 * Multiple types can be selected by combining type flags, in which case
 * the constructed DmaBufAllocator will match one of the types. If multiple
 * requested types can work on the system, which provider is used is undefined.
 */
DmaBufAllocator::DmaBufAllocator(DmaBufAllocatorFlags type)
{
	for (const auto &info : providerInfos) {
		if (!(type & info.type))
			continue;

		int ret = ::open(info.deviceNodeName, O_RDWR | O_CLOEXEC, 0);
		if (ret < 0) {
			ret = errno;
			LOG(DmaBufAllocator, Debug)
				<< "Failed to open " << info.deviceNodeName << ": "
				<< strerror(ret);
			continue;
		}

		LOG(DmaBufAllocator, Debug) << "Using " << info.deviceNodeName;
		providerHandle_ = UniqueFD(ret);
		type_ = info.type;
		break;
	}

	if (!providerHandle_.isValid())
		LOG(DmaBufAllocator, Error) << "Could not open any dma-buf provider";
}

/**
 * \brief Destroy the DmaBufAllocator instance
 */
DmaBufAllocator::~DmaBufAllocator() = default;

/**
 * \fn DmaBufAllocator::isValid()
 * \brief Check if the DmaBufAllocator instance is valid
 * \return True if the DmaBufAllocator is valid, false otherwise
 */

UniqueFD DmaBufAllocator::allocFromUDmaBuf(const char *name, std::size_t size)
{
	/* Size must be a multiple of the page size. Round it up. */
	std::size_t pageMask = sysconf(_SC_PAGESIZE) - 1;
	size = (size + pageMask) & ~pageMask;

	int ret = memfd_create(name, MFD_ALLOW_SEALING);
	if (ret < 0) {
		ret = errno;
		LOG(DmaBufAllocator, Error)
			<< "Failed to allocate memfd storage for " << name
			<< ": " << strerror(ret);
		return {};
	}

	UniqueFD memfd(ret);

	ret = ftruncate(memfd.get(), size);
	if (ret < 0) {
		ret = errno;
		LOG(DmaBufAllocator, Error)
			<< "Failed to set memfd size for " << name
			<< ": " << strerror(ret);
		return {};
	}

	/* udmabuf dma-buffers *must* have the F_SEAL_SHRINK seal. */
	ret = fcntl(memfd.get(), F_ADD_SEALS, F_SEAL_SHRINK);
	if (ret < 0) {
		ret = errno;
		LOG(DmaBufAllocator, Error)
			<< "Failed to seal the memfd for " << name
			<< ": " << strerror(ret);
		return {};
	}

	struct udmabuf_create create;

	create.memfd = memfd.get();
	create.flags = UDMABUF_FLAGS_CLOEXEC;
	create.offset = 0;
	create.size = size;

	ret = ::ioctl(providerHandle_.get(), UDMABUF_CREATE, &create);
	if (ret < 0) {
		ret = errno;
		LOG(DmaBufAllocator, Error)
			<< "Failed to create dma buf for " << name
			<< ": " << strerror(ret);
		return {};
	}

	/* The underlying memfd is kept as as a reference in the kernel. */
	return UniqueFD(ret);
}

UniqueFD DmaBufAllocator::allocFromHeap(const char *name, std::size_t size)
{
	struct dma_heap_allocation_data alloc = {};
	int ret;

	alloc.len = size;
	alloc.fd_flags = O_CLOEXEC | O_RDWR;

	ret = ::ioctl(providerHandle_.get(), DMA_HEAP_IOCTL_ALLOC, &alloc);
	if (ret < 0) {
		LOG(DmaBufAllocator, Error)
			<< "dma-heap allocation failure for " << name;
		return {};
	}

	UniqueFD allocFd(alloc.fd);
	ret = ::ioctl(allocFd.get(), DMA_BUF_SET_NAME, name);
	if (ret < 0) {
		LOG(DmaBufAllocator, Error)
			<< "dma-heap naming failure for " << name;
		return {};
	}

	return allocFd;
}

/**
 * \brief Allocate a dma-buf from the DmaBufAllocator
 * \param [in] name The name to set for the allocated buffer
 * \param [in] size The size of the buffer to allocate
 *
 * Allocates a dma-buf with read/write access.
 *
 * If the allocation fails, return an invalid UniqueFD.
 *
 * \return The UniqueFD of the allocated buffer
 */
UniqueFD DmaBufAllocator::alloc(const char *name, std::size_t size)
{
	if (!name)
		return {};

	if (type_ == DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf)
		return allocFromUDmaBuf(name, size);
	else
		return allocFromHeap(name, size);
}

} /* namespace libcamera */
