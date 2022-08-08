/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * mutex.h - Mutex classes with clang thread safety annotation
 */

#pragma once

#include <condition_variable>
#include <mutex>

#include <libcamera/base/private.h>

#include <libcamera/base/thread_annotations.h>

namespace libcamera {

/* \todo using Mutex = std::mutex if libc++ is used. */

#ifndef __DOXYGEN__

class LIBCAMERA_TSA_CAPABILITY("mutex") Mutex final
{
public:
	constexpr Mutex()
	{
	}

	void lock() LIBCAMERA_TSA_ACQUIRE()
	{
		mutex_.lock();
	}

	void unlock() LIBCAMERA_TSA_RELEASE()
	{
		mutex_.unlock();
	}

private:
	friend class MutexLocker;

	std::mutex mutex_;
};

class LIBCAMERA_TSA_SCOPED_CAPABILITY MutexLocker final
{
public:
	explicit MutexLocker(Mutex &mutex) LIBCAMERA_TSA_ACQUIRE(mutex)
		: lock_(mutex.mutex_)
	{
	}

	MutexLocker(Mutex &mutex, std::defer_lock_t t) noexcept LIBCAMERA_TSA_EXCLUDES(mutex)
		: lock_(mutex.mutex_, t)
	{
	}

	~MutexLocker() LIBCAMERA_TSA_RELEASE()
	{
	}

	void lock() LIBCAMERA_TSA_ACQUIRE()
	{
		lock_.lock();
	}

	bool try_lock() LIBCAMERA_TSA_TRY_ACQUIRE(true)
	{
		return lock_.try_lock();
	}

	void unlock() LIBCAMERA_TSA_RELEASE()
	{
		lock_.unlock();
	}

private:
	friend class ConditionVariable;

	std::unique_lock<std::mutex> lock_;
};

class ConditionVariable final
{
public:
	ConditionVariable()
	{
	}

	void notify_one() noexcept
	{
		cv_.notify_one();
	}

	void notify_all() noexcept
	{
		cv_.notify_all();
	}

	template<class Predicate>
	void wait(MutexLocker &locker, Predicate stopWaiting)
	{
		cv_.wait(locker.lock_, stopWaiting);
	}

	template<class Rep, class Period, class Predicate>
	bool wait_for(MutexLocker &locker,
		      const std::chrono::duration<Rep, Period> &relTime,
		      Predicate stopWaiting)
	{
		return cv_.wait_for(locker.lock_, relTime, stopWaiting);
	}

private:
	std::condition_variable cv_;
};

#else /* __DOXYGEN__ */

class Mutex final
{
};

class MutexLocker final
{
};

class ConditionVariable final
{
};

#endif /* __DOXYGEN__ */
} /* namespace libcamera */
