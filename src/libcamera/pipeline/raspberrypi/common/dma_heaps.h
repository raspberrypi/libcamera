/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * dma_heaps.h - Helper class for dma-heap allocations.
 */

#pragma once

#include <stddef.h>
#include <sys/mman.h>

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
	static constexpr std::size_t SIZE = sizeof(T);

	template<class... Args>
	DmaHeapObject(Args &&... args)
		: dmaHeap_(), fd_(), obj_(nullptr)
	{
		void *mem;

		fd_ = SharedFD(dmaHeap_.alloc("DmaHeapObject", SIZE));
		if (!fd_.isValid())
			return;

		mem = mmap(nullptr, SIZE, PROT_READ | PROT_WRITE,
			   MAP_SHARED, fd_.get(), 0);
		if (mem == MAP_FAILED)
			return;

		obj_ = new (mem) T(std::forward<Args>(args)...);
	}

	~DmaHeapObject()
	{
		if (obj_) {
			obj_->~T();
			munmap(obj_, SIZE);
		}
	}

	T *operator->()
	{
		return obj_;
	}

	const SharedFD &getFD() const
	{
		return fd_;
	}

private:
	DmaHeap dmaHeap_;
	SharedFD fd_;
	T *obj_;
};

} /* namespace RPi */

} /* namespace libcamera */
