/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#pragma once

#include <stddef.h>

#include <libcamera/base/unique_fd.h>

namespace libcamera {

namespace RPi {

class DmaHeap
{
public:
	DmaHeap();
	~DmaHeap();
	bool isValid() const { return dmaHeapHandle_.isValid(); }
	UniqueFD alloc(const char *name, std::size_t size);

private:
	UniqueFD dmaHeapHandle_;
};

} /* namespace RPi */

} /* namespace libcamera */
