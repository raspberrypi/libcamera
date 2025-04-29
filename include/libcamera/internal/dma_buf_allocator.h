/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * Helper class for dma-buf allocations.
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/base/flags.h>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class FrameBuffer;

class DmaBufAllocator
{
public:
	enum class DmaBufAllocatorFlag {
		CmaHeap = 1 << 0,
		SystemHeap = 1 << 1,
		UDmaBuf = 1 << 2,
	};

	using DmaBufAllocatorFlags = Flags<DmaBufAllocatorFlag>;

	DmaBufAllocator(DmaBufAllocatorFlags flags = DmaBufAllocatorFlag::CmaHeap);
	~DmaBufAllocator();
	bool isValid() const { return providerHandle_.isValid(); }
	UniqueFD alloc(const char *name, std::size_t size);

	int exportBuffers(unsigned int count,
			  const std::vector<unsigned int> &planeSizes,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

private:
	std::unique_ptr<FrameBuffer> createBuffer(
		std::string name, const std::vector<unsigned int> &planeSizes);

	UniqueFD allocFromHeap(const char *name, std::size_t size);
	UniqueFD allocFromUDmaBuf(const char *name, std::size_t size);
	UniqueFD providerHandle_;
	DmaBufAllocatorFlag type_;
};

class DmaSyncer final
{
public:
	enum class SyncType {
		Read = 0,
		Write,
		ReadWrite,
	};

	explicit DmaSyncer(SharedFD fd, SyncType type = SyncType::ReadWrite);

	DmaSyncer(DmaSyncer &&other) = default;
	DmaSyncer &operator=(DmaSyncer &&other) = default;

	~DmaSyncer();

private:
	LIBCAMERA_DISABLE_COPY(DmaSyncer)

	void sync(uint64_t step);

	SharedFD fd_;
	uint64_t flags_ = 0;
};

LIBCAMERA_FLAGS_ENABLE_OPERATORS(DmaBufAllocator::DmaBufAllocatorFlag)

} /* namespace libcamera */
