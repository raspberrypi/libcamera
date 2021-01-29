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
	event_base_loop(base_, EVLOOP_NO_EXIT_ON_EMPTY);
	return exitCode_;
}

void EventLoop::exit(int code)
{
	exitCode_ = code;
	event_base_loopbreak(base_);
}

void EventLoop::callLater(const std::function<void()> &func)
{
	{
		std::unique_lock<std::mutex> locker(lock_);
		calls_.push_back(func);
	}

	event_base_once(base_, -1, EV_TIMEOUT, dispatchCallback, this, nullptr);
}

void EventLoop::dispatchCallback([[maybe_unused]] evutil_socket_t fd,
				 [[maybe_unused]] short flags, void *param)
{
	EventLoop *loop = static_cast<EventLoop *>(param);
	loop->dispatchCall();
}

void EventLoop::dispatchCall()
{
	std::function<void()> call;

	{
		std::unique_lock<std::mutex> locker(lock_);
		if (calls_.empty())
			return;

		call = calls_.front();
		calls_.pop_front();
	}

	call();
}
