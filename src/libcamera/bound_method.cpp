/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * bound_method.cpp - Method bind and invocation
 */

#include <libcamera/bound_method.h>

#include "message.h"
#include "semaphore.h"
#include "thread.h"
#include "utils.h"

/**
 * \file bound_method.h
 * \brief Method bind and invocation
 */

namespace libcamera {

/**
 * \enum ConnectionType
 * \brief Connection type for asynchronous communication
 *
 * This enumeration describes the possible types of asynchronous communication
 * between a sender and a receiver. It applies to Signal::emit() and
 * Object::invokeMethod().
 *
 * \var ConnectionType::ConnectionTypeAuto
 * \brief If the sender and the receiver live in the same thread,
 * ConnectionTypeDirect is used. Otherwise ConnectionTypeQueued is used.
 *
 * \var ConnectionType::ConnectionTypeDirect
 * \brief The receiver is invoked immediately and synchronously in the sender's
 * thread.
 *
 * \var ConnectionType::ConnectionTypeQueued
 * \brief The receiver is invoked asynchronously in its thread when control
 * returns to the thread's event loop. The sender proceeds without waiting for
 * the invocation to complete.
 *
 * \var ConnectionType::ConnectionTypeBlocking
 * \brief The receiver is invoked asynchronously in its thread when control
 * returns to the thread's event loop. The sender blocks until the receiver
 * signals the completion of the invocation. This connection type shall not be
 * used when the sender and receiver live in the same thread, otherwise
 * deadlock will occur.
 */

void BoundMethodBase::activatePack(void *pack)
{
	ConnectionType type = connectionType_;
	if (type == ConnectionTypeAuto) {
		if (Thread::current() == object_->thread())
			type = ConnectionTypeDirect;
		else
			type = ConnectionTypeQueued;
	}

	switch (type) {
	case ConnectionTypeDirect:
	default:
		invokePack(pack);
		break;

	case ConnectionTypeQueued: {
		std::unique_ptr<Message> msg =
			utils::make_unique<InvokeMessage>(this, pack);
		object_->postMessage(std::move(msg));
		break;
	}

	case ConnectionTypeBlocking: {
		Semaphore semaphore;

		std::unique_ptr<Message> msg =
			utils::make_unique<InvokeMessage>(this, pack, &semaphore);
		object_->postMessage(std::move(msg));

		semaphore.acquire();
		break;
	}
	}
}

} /* namespace libcamera */
