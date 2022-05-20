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
#include <iostream>

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

	events_.clear();
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

void EventLoop::addFdEvent(int fd, EventType type,
			   const std::function<void()> &callback)
{
	std::unique_ptr<Event> event = std::make_unique<Event>(callback);
	short events = (type & Read ? EV_READ : 0)
		     | (type & Write ? EV_WRITE : 0)
		     | EV_PERSIST;

	event->event_ = event_new(base_, fd, events, &EventLoop::Event::dispatch,
				  event.get());
	if (!event->event_) {
		std::cerr << "Failed to create event for fd " << fd << std::endl;
		return;
	}

	int ret = event_add(event->event_, nullptr);
	if (ret < 0) {
		std::cerr << "Failed to add event for fd " << fd << std::endl;
		return;
	}

	events_.push_back(std::move(event));
}

void EventLoop::addTimerEvent(const std::chrono::microseconds period,
			      const std::function<void()> &callback)
{
	std::unique_ptr<Event> event = std::make_unique<Event>(callback);
	event->event_ = event_new(base_, -1, EV_PERSIST, &EventLoop::Event::dispatch,
				  event.get());
	if (!event->event_) {
		std::cerr << "Failed to create timer event" << std::endl;
		return;
	}

	struct timeval tv;
	tv.tv_sec = period.count() / 1000000ULL;
	tv.tv_usec = period.count() % 1000000ULL;

	int ret = event_add(event->event_, &tv);
	if (ret < 0) {
		std::cerr << "Failed to add timer event" << std::endl;
		return;
	}

	events_.push_back(std::move(event));
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

EventLoop::Event::Event(const std::function<void()> &callback)
	: callback_(callback), event_(nullptr)
{
}

EventLoop::Event::~Event()
{
	event_del(event_);
	event_free(event_);
}

void EventLoop::Event::dispatch([[maybe_unused]] int fd,
				[[maybe_unused]] short events, void *arg)
{
	Event *event = static_cast<Event *>(arg);
	event->callback_();
}
