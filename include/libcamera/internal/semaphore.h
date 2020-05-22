/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * semaphore.h - General-purpose counting semaphore
 */
#ifndef __LIBCAMERA_INTERNAL_SEMAPHORE_H__
#define __LIBCAMERA_INTERNAL_SEMAPHORE_H__

#include <condition_variable>

#include "libcamera/internal/thread.h"

namespace libcamera {

class Semaphore
{
public:
	Semaphore(unsigned int n = 0);

	unsigned int available();
	void acquire(unsigned int n = 1);
	bool tryAcquire(unsigned int n = 1);
	void release(unsigned int n = 1);

private:
	Mutex mutex_;
	std::condition_variable cv_;
	unsigned int available_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_SEMAPHORE_H__ */
