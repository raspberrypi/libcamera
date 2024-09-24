/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 * Copyright (C) 2024 Andrei Konovalov
 * Copyright (C) 2024 Dennis Bonke
 *
 * Helpers for shared memory allocations
 */
#pragma once

#include <stdint.h>
#include <string>
#include <sys/mman.h>
#include <type_traits>
#include <utility>

#include <libcamera/base/class.h>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/span.h>

namespace libcamera {

class SharedMem
{
public:
	SharedMem();

	SharedMem(const std::string &name, std::size_t size);
	SharedMem(SharedMem &&rhs);

	virtual ~SharedMem();

	SharedMem &operator=(SharedMem &&rhs);

	const SharedFD &fd() const
	{
		return fd_;
	}

	Span<uint8_t> mem() const
	{
		return mem_;
	}

	explicit operator bool() const
	{
		return !mem_.empty();
	}

private:
	LIBCAMERA_DISABLE_COPY(SharedMem)

	SharedFD fd_;

	Span<uint8_t> mem_;
};

template<class T, typename = std::enable_if_t<std::is_standard_layout<T>::value>>
class SharedMemObject : public SharedMem
{
public:
	static constexpr std::size_t kSize = sizeof(T);

	SharedMemObject()
		: SharedMem(), obj_(nullptr)
	{
	}

	template<class... Args>
	SharedMemObject(const std::string &name, Args &&...args)
		: SharedMem(name, kSize), obj_(nullptr)
	{
		if (mem().empty())
			return;

		obj_ = new (mem().data()) T(std::forward<Args>(args)...);
	}

	SharedMemObject(SharedMemObject<T> &&rhs)
		: SharedMem(std::move(rhs))
	{
		this->obj_ = rhs.obj_;
		rhs.obj_ = nullptr;
	}

	~SharedMemObject()
	{
		if (obj_)
			obj_->~T();
	}

	SharedMemObject<T> &operator=(SharedMemObject<T> &&rhs)
	{
		SharedMem::operator=(std::move(rhs));
		this->obj_ = rhs.obj_;
		rhs.obj_ = nullptr;
		return *this;
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

private:
	LIBCAMERA_DISABLE_COPY(SharedMemObject)

	T *obj_;
};

} /* namespace libcamera */
