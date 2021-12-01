/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * semaphore.h - General-purpose counting semaphore
 */

#pragma once

#include <libcamera/base/private.h>

#include <libcamera/base/mutex.h>

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
	ConditionVariable cv_;
	unsigned int available_;
};

} /* namespace libcamera */
