/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * bound_method.cpp - Method bind and invocation
 */

#include <libcamera/bound_method.h>

#include "message.h"
#include "thread.h"
#include "utils.h"

namespace libcamera {

void BoundMethodBase::activatePack(void *pack)
{
	if (Thread::current() == object_->thread()) {
		invokePack(pack);
	} else {
		std::unique_ptr<Message> msg =
			utils::make_unique<SignalMessage>(this, pack);
		object_->postMessage(std::move(msg));
	}
}

} /* namespace libcamera */
