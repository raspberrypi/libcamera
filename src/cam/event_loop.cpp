/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_loop.cpp - cam - Event loop
 */

#include <libcamera/event_dispatcher.h>

#include "event_loop.h"

using namespace libcamera;

EventLoop::EventLoop(EventDispatcher *dispatcher)
	: dispatcher_(dispatcher)
{
}

EventLoop::~EventLoop()
{
}

int EventLoop::exec()
{
	exitCode_ = -1;
	exit_.store(false, std::memory_order_release);

	while (!exit_.load(std::memory_order_acquire))
		dispatcher_->processEvents();

	return exitCode_;
}

void EventLoop::exit(int code)
{
	exitCode_ = code;
	exit_.store(true, std::memory_order_release);
	dispatcher_->interrupt();
}
