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

#include "message.h"
#include "thread.h"

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

class ThreadRpcMessage : public libcamera::Message
{
public:
	ThreadRpcMessage();
	ThreadRpc *rpc;

	static Message::Type type();

private:
	static libcamera::Message::Type rpcType_;
};

#endif /* __ANDROID_THREAD_RPC_H__ */
