/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * shared_mem_object.h - Helper class for shared memory allocations
 */
#pragma once

#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <libcamera/base/shared_fd.h>

namespace libcamera {

namespace RPi {

template<class T>
class SharedMemObject
{
public:
	static constexpr std::size_t SIZE = sizeof(T);

	template<class... Args>
	SharedMemObject(const std::string &name, Args &&...args)
		: name_(name), obj_(nullptr)
	{
		void *mem;
		int ret;

		ret = memfd_create(name_.c_str(), MFD_CLOEXEC);
		if (ret < 0)
			return;

		fd_ = SharedFD(ret);
		if (!fd_.isValid())
			return;

		ftruncate(fd_.get(), SIZE);
		mem = mmap(nullptr, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			   fd_.get(), 0);
		if (mem == MAP_FAILED)
			return;

		obj_ = new (mem) T(std::forward<Args>(args)...);
	}

	~SharedMemObject()
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

	const T *operator->() const
	{
		return obj_;
	}

	T &operator*()
	{
		return *obj_;
	}

	const T &operator*() const
	{
		return *obj_;
	}

	const SharedFD &getFD() const
	{
		return fd_;
	}

	explicit operator bool() const
	{
		return !!obj_;
	}

private:
	std::string name_;
	SharedFD fd_;
	T *obj_;
};

} /* namespace RPi */

} /* namespace libcamera */
