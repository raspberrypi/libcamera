/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * metadata.h - general metadata class
 */
#pragma once

/* A simple class for carrying arbitrary metadata, for example about an image. */

#include <any>
#include <map>
#include <mutex>
#include <string>

#include <libcamera/base/thread_annotations.h>

namespace RPiController {

class LIBCAMERA_TSA_CAPABILITY("mutex") Metadata
{
public:
	Metadata() = default;

	Metadata(Metadata const &other)
	{
		std::scoped_lock otherLock(other.mutex_);
		data_ = other.data_;
	}

	Metadata(Metadata &&other)
	{
		std::scoped_lock otherLock(other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
	}

	template<typename T>
	void set(std::string const &tag, T const &value)
	{
		std::scoped_lock lock(mutex_);
		data_[tag] = value;
	}

	template<typename T>
	int get(std::string const &tag, T &value) const
	{
		std::scoped_lock lock(mutex_);
		auto it = data_.find(tag);
		if (it == data_.end())
			return -1;
		value = std::any_cast<T>(it->second);
		return 0;
	}

	void clear()
	{
		std::scoped_lock lock(mutex_);
		data_.clear();
	}

	Metadata &operator=(Metadata const &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_ = other.data_;
		return *this;
	}

	Metadata &operator=(Metadata &&other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
		return *this;
	}

	void merge(Metadata &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_.merge(other.data_);
	}

	void mergeCopy(const Metadata &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		/*
		 * If the metadata key exists, ignore this item and copy only
		 * unique key/value pairs.
		 */
		data_.insert(other.data_.begin(), other.data_.end());
	}

	template<typename T>
	T *getLocked(std::string const &tag)
	{
		/*
		 * This allows in-place access to the Metadata contents,
		 * for which you should be holding the lock.
		 */
		auto it = data_.find(tag);
		if (it == data_.end())
			return nullptr;
		return std::any_cast<T>(&it->second);
	}

	template<typename T>
	void setLocked(std::string const &tag, T const &value)
	{
		/* Use this only if you're holding the lock yourself. */
		data_[tag] = value;
	}

	/*
	 * Note: use of (lowercase) lock and unlock means you can create scoped
	 * locks with the standard lock classes.
	 * e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
	 */
	void lock() LIBCAMERA_TSA_ACQUIRE() { mutex_.lock(); }
	void unlock() LIBCAMERA_TSA_RELEASE() { mutex_.unlock(); }

private:
	mutable std::mutex mutex_;
	std::map<std::string, std::any> data_;
};

} /* namespace RPiController */
