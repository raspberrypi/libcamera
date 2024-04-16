/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * dma_heaps.cpp - Helper class for dma-heap allocations.
 */

#include "libcamera/internal/dma_heaps.h"

#include <array>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include <libcamera/base/log.h>

/**
 * \file dma_heaps.cpp
 * \brief CMA dma-heap allocator
 */

/*
 * /dev/dma_heap/linux,cma is the dma-heap allocator, which allows dmaheap-cma
 * to only have to worry about importing.
 *
 * Annoyingly, should the cma heap size be specified on the kernel command line
 * instead of DT, the heap gets named "reserved" instead.
 */
static constexpr std::array<const char *, 2> heapNames = {
	"/dev/dma_heap/linux,cma",
	"/dev/dma_heap/reserved"
};

namespace libcamera {

LOG_DEFINE_CATEGORY(DmaHeap)

/**
 * \class DmaHeap
 * \brief Helper class for CMA dma-heap allocations
 */

/**
 * \brief Construct a DmaHeap that owns a CMA dma-heap file descriptor
 *
 * Looks for a CMA dma-heap device to use. If it fails to open any dma-heap
 * device, an invalid DmaHeap object is constructed.
 *
 * Check the new DmaHeap object with isValid before using it.
 */
DmaHeap::DmaHeap()
{
	for (const char *name : heapNames) {
		int ret = ::open(name, O_RDWR | O_CLOEXEC, 0);
		if (ret < 0) {
			ret = errno;
			LOG(DmaHeap, Debug)
				<< "Failed to open " << name << ": "
				<< strerror(ret);
			continue;
		}

		dmaHeapHandle_ = UniqueFD(ret);
		break;
	}

	if (!dmaHeapHandle_.isValid())
		LOG(DmaHeap, Error) << "Could not open any dmaHeap device";
}

/**
 * \brief Destroy the DmaHeap instance
 */
DmaHeap::~DmaHeap() = default;

/**
 * \fn DmaHeap::isValid()
 * \brief Check if the DmaHeap instance is valid
 * \return True if the DmaHeap is valid, false otherwise
 */

/**
 * \brief Allocate a dma-buf from the DmaHeap
 * \param [in] name The name to set for the allocated buffer
 * \param [in] size The size of the buffer to allocate
 *
 * Allocates a dma-buf with read/write access.
 *
 * If the allocation fails, return an invalid UniqueFD.
 *
 * \return The UniqueFD of the allocated buffer
 */
UniqueFD DmaHeap::alloc(const char *name, std::size_t size)
{
	int ret;

	if (!name)
		return {};

	struct dma_heap_allocation_data alloc = {};

	alloc.len = size;
	alloc.fd_flags = O_CLOEXEC | O_RDWR;

	ret = ::ioctl(dmaHeapHandle_.get(), DMA_HEAP_IOCTL_ALLOC, &alloc);
	if (ret < 0) {
		LOG(DmaHeap, Error) << "dmaHeap allocation failure for " << name;
		return {};
	}

	UniqueFD allocFd(alloc.fd);
	ret = ::ioctl(allocFd.get(), DMA_BUF_SET_NAME, name);
	if (ret < 0) {
		LOG(DmaHeap, Error) << "dmaHeap naming failure for " << name;
		return {};
	}

	return allocFd;
}

} /* namespace libcamera */
