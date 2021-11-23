/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#pragma once

#include <libcamera/file_descriptor.h>

namespace libcamera {

namespace RPi {

class DmaHeap
{
public:
	DmaHeap();
	~DmaHeap();
	bool isValid() const { return dmaHeapHandle_ > -1; }
	FileDescriptor alloc(const char *name, std::size_t size);

private:
	int dmaHeapHandle_;
};

} /* namespace RPi */

} /* namespace libcamera */
