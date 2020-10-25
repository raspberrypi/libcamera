/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_loop.cpp - cam - Event loop
 */

#include "event_loop.h"

#include <assert.h>
#include <event2/event.h>
#include <event2/thread.h>

EventLoop *EventLoop::instance_ = nullptr;

EventLoop::EventLoop()
{
	assert(!instance_);

	evthread_use_pthreads();
	event_ = event_base_new();
	instance_ = this;
}

EventLoop::~EventLoop()
{
	instance_ = nullptr;

	event_base_free(event_);
	libevent_global_shutdown();
}

EventLoop *EventLoop::instance()
{
	return instance_;
}

int EventLoop::exec()
{
	exitCode_ = -1;
	exit_.store(false, std::memory_order_release);

	while (!exit_.load(std::memory_order_acquire))
		event_base_loop(event_, EVLOOP_NO_EXIT_ON_EMPTY);

	return exitCode_;
}

void EventLoop::exit(int code)
{
	exitCode_ = code;
	exit_.store(true, std::memory_order_release);
	interrupt();
}

void EventLoop::interrupt()
{
	event_base_loopbreak(event_);
}
