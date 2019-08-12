/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread_rpc.h - Inter-thread procedure call
 */
#ifndef __ANDROID_THREAD_RPC_H__
#define __ANDROID_THREAD_RPC_H__

#include <condition_variable>
#include <mutex>

#include <hardware/camera3.h>

class ThreadRpc
{
public:
	enum RpcTag {
		ProcessCaptureRequest,
		Close,
	};

	ThreadRpc()
		: delivered_(false) {}

	void notifyReception();
	void waitDelivery();

	RpcTag tag;

	camera3_capture_request_t *request;

private:
	bool delivered_;
	std::mutex mutex_;
	std::condition_variable cv_;
};

#endif /* __ANDROID_THREAD_RPC_H__ */
