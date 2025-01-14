/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * cam - Event loop
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

	callsTrigger_ = event_new(base_, -1, EV_PERSIST, [](evutil_socket_t, short, void *closure) {
		auto *self = static_cast<EventLoop *>(closure);

		for (;;) {
			std::function<void()> call;

			{
				std::lock_guard locker(self->lock_);
				if (self->calls_.empty())
					break;

				call = std::move(self->calls_.front());
				self->calls_.pop_front();
			}

			call();
		}
	}, this);
	assert(callsTrigger_);
	event_add(callsTrigger_, nullptr);
}

EventLoop::~EventLoop()
{
	instance_ = nullptr;

	event_free(callsTrigger_);

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

void EventLoop::callLater(std::function<void()> &&func)
{
	{
		std::unique_lock<std::mutex> locker(lock_);
		calls_.push_back(std::move(func));
	}

	event_active(callsTrigger_, 0, 0);
}

void EventLoop::addFdEvent(int fd, EventType type,
			   std::function<void()> &&callback)
{
	std::unique_ptr<Event> event = std::make_unique<Event>(std::move(callback));
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
			      std::function<void()> &&callback)
{
	std::unique_ptr<Event> event = std::make_unique<Event>(std::move(callback));
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

EventLoop::Event::Event(std::function<void()> &&callback)
	: callback_(std::move(callback)), event_(nullptr)
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
