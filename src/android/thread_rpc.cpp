/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread_rpc.cpp - Inter-thread procedure call
 */

#include "thread_rpc.h"

#include "message.h"

using namespace libcamera;

libcamera::Message::Type ThreadRpcMessage::rpcType_ = Message::Type::None;

ThreadRpcMessage::ThreadRpcMessage()
	: Message(type())
{
}

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

Message::Type ThreadRpcMessage::type()
{
	if (ThreadRpcMessage::rpcType_ == Message::Type::None)
		rpcType_ = Message::registerMessageType();

	return rpcType_;
}
