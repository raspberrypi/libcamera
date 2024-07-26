/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * Helper class for dma-buf allocations.
 */

#pragma once

#include <stddef.h>

#include <libcamera/base/flags.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

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

private:
	UniqueFD allocFromHeap(const char *name, std::size_t size);
	UniqueFD allocFromUDmaBuf(const char *name, std::size_t size);
	UniqueFD providerHandle_;
	DmaBufAllocatorFlag type_;
};

LIBCAMERA_FLAGS_ENABLE_OPERATORS(DmaBufAllocator::DmaBufAllocatorFlag)

} /* namespace libcamera */
