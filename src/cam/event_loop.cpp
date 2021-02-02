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
	base_ = event_base_new();
	instance_ = this;
}

EventLoop::~EventLoop()
{
	instance_ = nullptr;

	event_base_free(base_);
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

	while (!exit_.load(std::memory_order_acquire)) {
		dispatchCalls();
		event_base_loop(base_, EVLOOP_NO_EXIT_ON_EMPTY);
	}

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
	event_base_loopbreak(base_);
}

void EventLoop::callLater(const std::function<void()> &func)
{
	{
		std::unique_lock<std::mutex> locker(lock_);
		calls_.push_back(func);
	}

	interrupt();
}

void EventLoop::dispatchCalls()
{
	std::unique_lock<std::mutex> locker(lock_);

	for (auto iter = calls_.begin(); iter != calls_.end(); ) {
		std::function<void()> call = std::move(*iter);

		iter = calls_.erase(iter);

		locker.unlock();
		call();
		locker.lock();
	}
}
