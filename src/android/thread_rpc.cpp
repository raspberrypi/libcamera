/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread_rpc.cpp - Inter-thread procedure call
 */

#include "thread.h"
#include "thread_rpc.h"

using namespace libcamera;

void ThreadRpc::notifyReception()
{
	{
		libcamera::MutexLocker locker(mutex_);
		delivered_ = true;
	}
	cv_.notify_one();
}

void ThreadRpc::waitDelivery()
{
	libcamera::MutexLocker locker(mutex_);
	cv_.wait(locker, [&] { return delivered_; });
}
