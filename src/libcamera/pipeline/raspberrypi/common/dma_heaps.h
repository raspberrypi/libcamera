/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#pragma once

#include <stddef.h>
#include <sys/mman.h>

//#include <libcamera/logging.h>

#include <libcamera/base/shared_fd.h>
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

template<class T>
class DmaHeapObject
{
public:
	template<class... Args>
	DmaHeapObject(Args... args)
		: dmaHeap_(), obj_(nullptr), fd_()
	{
		void *mem;

		//ASSERT(dmaHeap_->isValid());

		fd_ = dmaHeap_.alloc("DmaHeapObject", sizeof(T));
		if (!fd_.isValid())
			return;

		mem = mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE,
			   MAP_SHARED, fd_.get(), 0);

		if (mem == MAP_FAILED)
			return;

		obj_ = new (mem) T(std::forward<Args>(args)...);
	}

	~DmaHeapObject()
	{
		if (obj_) {
			obj_->~T();
			munmap(obj_, sizeof(T));
		}
	}

	T *operator->()
	{
		return obj_;
	}

private:
	DmaHeap dmaHeap_;
	T *obj_;
	SharedFD fd_;
};

} /* namespace RPi */

} /* namespace libcamera */
