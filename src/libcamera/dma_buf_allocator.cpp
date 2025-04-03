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
#include <libcamera/base/memfd.h>
#include <libcamera/base/shared_fd.h>

#include <libcamera/framebuffer.h>

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

	/* udmabuf dma-buffers *must* have the F_SEAL_SHRINK seal. */
	UniqueFD memfd = MemFd::create(name, size, MemFd::Seal::Shrink);
	if (!memfd.isValid())
		return {};

	struct udmabuf_create create;

	create.memfd = memfd.get();
	create.flags = UDMABUF_FLAGS_CLOEXEC;
	create.offset = 0;
	create.size = size;

	int ret = ::ioctl(providerHandle_.get(), UDMABUF_CREATE, &create);
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

/**
 * \brief Allocate and export buffers from the DmaBufAllocator
 * \param[in] count The number of requested FrameBuffers
 * \param[in] planeSizes The sizes of planes in each FrameBuffer
 * \param[out] buffers Array of buffers successfully allocated
 *
 * Planes in a FrameBuffer are allocated with a single dma buf.
 * \todo Add the option to allocate each plane with a dma buf respectively.
 *
 * \return The number of allocated buffers on success or a negative error code
 * otherwise
 */
int DmaBufAllocator::exportBuffers(unsigned int count,
				   const std::vector<unsigned int> &planeSizes,
				   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	for (unsigned int i = 0; i < count; ++i) {
		std::unique_ptr<FrameBuffer> buffer =
			createBuffer("frame-" + std::to_string(i), planeSizes);
		if (!buffer) {
			LOG(DmaBufAllocator, Error) << "Unable to create buffer";

			buffers->clear();
			return -EINVAL;
		}

		buffers->push_back(std::move(buffer));
	}

	return count;
}

std::unique_ptr<FrameBuffer>
DmaBufAllocator::createBuffer(std::string name,
			      const std::vector<unsigned int> &planeSizes)
{
	std::vector<FrameBuffer::Plane> planes;

	unsigned int frameSize = 0, offset = 0;
	for (auto planeSize : planeSizes)
		frameSize += planeSize;

	SharedFD fd(alloc(name.c_str(), frameSize));
	if (!fd.isValid())
		return nullptr;

	for (auto planeSize : planeSizes) {
		planes.emplace_back(FrameBuffer::Plane{ fd, offset, planeSize });
		offset += planeSize;
	}

	return std::make_unique<FrameBuffer>(planes);
}

/**
 * \class DmaSyncer
 * \brief Helper class for dma-buf's synchronization
 *
 * This class wraps a userspace dma-buf's synchronization process with an
 * object's lifetime.
 *
 * It's used when the user needs to access a dma-buf with CPU, mostly mapped
 * with MappedFrameBuffer, so that the buffer is synchronized between CPU and
 * ISP.
 */

/**
 * \enum DmaSyncer::SyncType
 * \brief Read and/or write access via the CPU map
 * \var DmaSyncer::Read
 * \brief Indicates that the mapped dma-buf will be read by the client via the
 * CPU map
 * \var DmaSyncer::Write
 * \brief Indicates that the mapped dm-buf will be written by the client via the
 * CPU map
 * \var DmaSyncer::ReadWrite
 * \brief Indicates that the mapped dma-buf will be read and written by the
 * client via the CPU map
 */

/**
 * \brief Construct a DmaSyncer with a dma-buf's fd and the access type
 * \param[in] fd The dma-buf's file descriptor to synchronize
 * \param[in] type Read and/or write access via the CPU map
 */
DmaSyncer::DmaSyncer(SharedFD fd, SyncType type)
	: fd_(fd)
{
	switch (type) {
	case SyncType::Read:
		flags_ = DMA_BUF_SYNC_READ;
		break;
	case SyncType::Write:
		flags_ = DMA_BUF_SYNC_WRITE;
		break;
	case SyncType::ReadWrite:
		flags_ = DMA_BUF_SYNC_RW;
		break;
	}

	sync(DMA_BUF_SYNC_START);
}

/**
 * \fn DmaSyncer::DmaSyncer(DmaSyncer &&other);
 * \param[in] other The other instance
 * \brief Enable move on class DmaSyncer
 */

/**
 * \fn DmaSyncer::operator=(DmaSyncer &&other);
 * \param[in] other The other instance
 * \brief Enable move on class DmaSyncer
 */

DmaSyncer::~DmaSyncer()
{
	/*
	 * DmaSyncer might be moved and left with an empty SharedFD.
	 * Avoid syncing with an invalid file descriptor in this case.
	 */
	if (fd_.isValid())
		sync(DMA_BUF_SYNC_END);
}

void DmaSyncer::sync(uint64_t step)
{
	struct dma_buf_sync sync = {
		.flags = flags_ | step
	};

	int ret;
	do {
		ret = ioctl(fd_.get(), DMA_BUF_IOCTL_SYNC, &sync);
	} while (ret && (errno == EINTR || errno == EAGAIN));

	if (ret) {
		ret = errno;
		LOG(DmaBufAllocator, Error)
			<< "Unable to sync dma fd: " << fd_.get()
			<< ", err: " << strerror(ret)
			<< ", flags: " << sync.flags;
	}
}

} /* namespace libcamera */
