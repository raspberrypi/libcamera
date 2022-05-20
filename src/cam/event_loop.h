/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_loop.h - cam - Event loop
 */

#pragma once

#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <mutex>

#include <event2/util.h>

struct event_base;

class EventLoop
{
public:
	enum EventType {
		Read = 1,
		Write = 2,
	};

	EventLoop();
	~EventLoop();

	static EventLoop *instance();

	int exec();
	void exit(int code = 0);

	void callLater(const std::function<void()> &func);

	void addFdEvent(int fd, EventType type,
			const std::function<void()> &handler);

	using duration = std::chrono::steady_clock::duration;
	void addTimerEvent(const std::chrono::microseconds period,
			   const std::function<void()> &handler);

private:
	struct Event {
		Event(const std::function<void()> &callback);
		~Event();

		static void dispatch(int fd, short events, void *arg);

		std::function<void()> callback_;
		struct event *event_;
	};

	static EventLoop *instance_;

	struct event_base *base_;
	int exitCode_;

	std::list<std::function<void()>> calls_;
	std::list<std::unique_ptr<Event>> events_;
	std::mutex lock_;

	static void dispatchCallback(evutil_socket_t fd, short flags,
				     void *param);
	void dispatchCall();
};
