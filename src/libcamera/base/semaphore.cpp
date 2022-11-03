/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * semaphore.cpp - General-purpose counting semaphore
 */

#include <libcamera/base/semaphore.h>

/**
 * \file base/semaphore.h
 * \brief General-purpose counting semaphore
 */

namespace libcamera {

/**
 * \class Semaphore
 * \brief General-purpose counting semaphore
 *
 * A semaphore is a locking primitive that protects resources. It is created
 * with an initial number of resources (which may be 0), and offers two
 * primitives to acquire and release resources. The acquire() function tries to
 * acquire a number of resources, and blocks if not enough resources are
 * available until they get released. The release() function releases a number
 * of resources, waking up any consumer blocked on an acquire() call.
 */

/**
 * \brief Construct a semaphore with \a n resources
 * \param[in] n The resource count
 */
Semaphore::Semaphore(unsigned int n)
	: available_(n)
{
}

/**
 * \brief Retrieve the number of available resources
 * \return The number of available resources
 */
unsigned int Semaphore::available()
{
	MutexLocker locker(mutex_);
	return available_;
}

/**
 * \brief Acquire \a n resources
 * \param[in] n The resource count
 *
 * This function attempts to acquire \a n resources. If \a n is higher than the
 * number of available resources, the call will block until enough resources
 * become available.
 */
void Semaphore::acquire(unsigned int n)
{
	MutexLocker locker(mutex_);
	cv_.wait(locker, [&]() LIBCAMERA_TSA_REQUIRES(mutex_) {
		return available_ >= n;
	});
	available_ -= n;
}

/**
 * \brief Try to acquire \a n resources without blocking
 * \param[in] n The resource count
 *
 * This function attempts to acquire \a n resources. If \a n is higher than the
 * number of available resources, it returns false immediately without
 * acquiring any resource. Otherwise it acquires the resources and returns
 * true.
 *
 * \return True if the resources have been acquired, false otherwise
 */
bool Semaphore::tryAcquire(unsigned int n)
{
	MutexLocker locker(mutex_);
	if (available_ < n)
		return false;

	available_ -= n;
	return true;
}

/**
 * \brief Release \a n resources
 * \param[in] n The resource count
 *
 * This function releases \a n resources, increasing the available resource
 * count by \a n. If the number of available resources becomes large enough for
 * any consumer blocked on an acquire() call, those consumers get woken up.
 */
void Semaphore::release(unsigned int n)
{
	{
		MutexLocker locker(mutex_);
		available_ += n;
	}

	cv_.notify_all();
}

} /* namespace libcamera */
