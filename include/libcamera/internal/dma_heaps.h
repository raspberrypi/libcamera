/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#pragma once

#include <stddef.h>

#include <libcamera/base/flags.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class DmaHeap
{
public:
	enum class DmaHeapFlag {
		Cma = 1 << 0,
		System = 1 << 1,
	};

	using DmaHeapFlags = Flags<DmaHeapFlag>;

	DmaHeap(DmaHeapFlags flags = DmaHeapFlag::Cma);
	~DmaHeap();
	bool isValid() const { return dmaHeapHandle_.isValid(); }
	UniqueFD alloc(const char *name, std::size_t size);

private:
	UniqueFD dmaHeapHandle_;
};

LIBCAMERA_FLAGS_ENABLE_OPERATORS(DmaHeap::DmaHeapFlag)

} /* namespace libcamera */
