/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * metadata.hpp - general metadata class
 */
#pragma once

// A simple class for carrying arbitrary metadata, for example about an image.

#include <string>
#include <mutex>
#include <map>
#include <memory>

#include <boost/any.hpp>

namespace RPi {

class Metadata
{
public:
	template<typename T> void Set(std::string const &tag, T const &value)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		data_[tag] = value;
	}
	template<typename T> int Get(std::string const &tag, T &value) const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		auto it = data_.find(tag);
		if (it == data_.end())
			return -1;
		value = boost::any_cast<T>(it->second);
		return 0;
	}
	void Clear()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		data_.clear();
	}
	Metadata &operator=(Metadata const &other)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		std::lock_guard<std::mutex> other_lock(other.mutex_);
		data_ = other.data_;
		return *this;
	}
	template<typename T> T *GetLocked(std::string const &tag)
	{
		// This allows in-place access to the Metadata contents,
		// for which you should be holding the lock.
		auto it = data_.find(tag);
		if (it == data_.end())
			return nullptr;
		return boost::any_cast<T>(&it->second);
	}
	template<typename T>
	void SetLocked(std::string const &tag, T const &value)
	{
		// Use this only if you're holding the lock yourself.
		data_[tag] = value;
	}
	// Note: use of (lowercase) lock and unlock means you can create scoped
	// locks with the standard lock classes.
	// e.g. std::lock_guard<PisP::Metadata> lock(metadata)
	void lock() { mutex_.lock(); }
	void unlock() { mutex_.unlock(); }

private:
	mutable std::mutex mutex_;
	std::map<std::string, boost::any> data_;
};

typedef std::shared_ptr<Metadata> MetadataPtr;

} // namespace RPi
