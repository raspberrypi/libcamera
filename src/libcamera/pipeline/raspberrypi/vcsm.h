/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * vcsm.h - Helper class for vcsm allocations.
 */
#ifndef __LIBCAMERA_PIPELINE_RASPBERRYPI_VCSM_H__
#define __LIBCAMERA_PIPELINE_RASPBERRYPI_VCSM_H__

#include <iostream>
#include <mutex>

#include <fcntl.h>
#include <linux/vc_sm_cma_ioctl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace RPi {

#define VCSM_CMA_DEVICE_NAME "/dev/vcsm-cma"

class Vcsm
{
public:
	Vcsm()
	{
		vcsmHandle_ = ::open(VCSM_CMA_DEVICE_NAME, O_RDWR, 0);
		if (vcsmHandle_ == -1) {
			std::cerr << "Could not open vcsm device: "
				  << VCSM_CMA_DEVICE_NAME;
		}
	}

	~Vcsm()
	{
		/* Free all existing allocations. */
		auto it = allocMap_.begin();
		while (it != allocMap_.end())
			it = remove(it->first);

		if (vcsmHandle_)
			::close(vcsmHandle_);
	}

	void *alloc(const char *name, unsigned int size,
		    vc_sm_cma_cache_e cache = VC_SM_CMA_CACHE_NONE)
	{
		unsigned int pageSize = getpagesize();
		void *user_ptr;
		int ret;

		if (!name)
			return nullptr;

		/* Ask for page aligned allocation. */
		size = (size + pageSize - 1) & ~(pageSize - 1);

		struct vc_sm_cma_ioctl_alloc alloc;
		memset(&alloc, 0, sizeof(alloc));
		alloc.size = size;
		alloc.num = 1;
		alloc.cached = cache;
		alloc.handle = 0;
		memcpy(alloc.name, name, 32);

		ret = ::ioctl(vcsmHandle_, VC_SM_CMA_IOCTL_MEM_ALLOC, &alloc);

		if (ret < 0 || alloc.handle < 0) {
			std::cerr << "vcsm allocation failure for "
				  << name << std::endl;
			return nullptr;
		}

		/* Map the buffer into user space. */
		user_ptr = ::mmap(0, alloc.size, PROT_READ | PROT_WRITE,
				  MAP_SHARED, alloc.handle, 0);

		if (user_ptr == MAP_FAILED) {
			std::cerr << "vcsm mmap failure for " << name << std::endl;
			::close(alloc.handle);
			return nullptr;
		}

		std::lock_guard<std::mutex> lock(lock_);
		allocMap_.emplace(user_ptr, AllocInfo(alloc.handle,
						      alloc.size, alloc.vc_handle));

		return user_ptr;
	}

	void free(void *user_ptr)
	{
		std::lock_guard<std::mutex> lock(lock_);
		remove(user_ptr);
	}

	unsigned int getVCHandle(void *user_ptr)
	{
		std::lock_guard<std::mutex> lock(lock_);
		auto it = allocMap_.find(user_ptr);
		if (it != allocMap_.end())
			return it->second.vcHandle;

		return 0;
	}

private:
	struct AllocInfo {
		AllocInfo(int handle_, int size_, int vcHandle_)
			: handle(handle_), size(size_), vcHandle(vcHandle_)
		{
		}

		int handle;
		int size;
		uint32_t vcHandle;
	};

	/* Map of all allocations that have been requested. */
	using AllocMap = std::map<void *, AllocInfo>;

	AllocMap::iterator remove(void *user_ptr)
	{
		auto it = allocMap_.find(user_ptr);
		if (it != allocMap_.end()) {
			int handle = it->second.handle;
			int size = it->second.size;
			::munmap(user_ptr, size);
			::close(handle);
			/*
			 * Remove the allocation from the map. This returns
			 * an iterator to the next element.
			 */
			it = allocMap_.erase(it);
		}

		/* Returns an iterator to the next element. */
		return it;
	}

	AllocMap allocMap_;
	int vcsmHandle_;
	std::mutex lock_;
};

} /* namespace RPi */

#endif /* __LIBCAMERA_PIPELINE_RASPBERRYPI_VCSM_H__ */
