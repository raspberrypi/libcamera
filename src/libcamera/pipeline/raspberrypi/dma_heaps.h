/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */
#ifndef __LIBCAMERA_PIPELINE_RASPBERRYPI_DMA_HEAPS_H__
#define __LIBCAMERA_PIPELINE_RASPBERRYPI_DMA_HEAPS_H__

#include <libcamera/file_descriptor.h>

namespace libcamera {

namespace RPi {

class DmaHeap
{
public:
	DmaHeap();
	~DmaHeap();
	FileDescriptor alloc(const char *name, std::size_t size);

private:
	int dmaHeapHandle_;
};

} /* namespace RPi */

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_RASPBERRYPI_DMA_HEAPS_H__ */
