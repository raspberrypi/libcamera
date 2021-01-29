/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_loop.h - cam - Event loop
 */
#ifndef __CAM_EVENT_LOOP_H__
#define __CAM_EVENT_LOOP_H__

#include <functional>
#include <list>
#include <mutex>

#include <event2/util.h>

struct event_base;

class EventLoop
{
public:
	EventLoop();
	~EventLoop();

	static EventLoop *instance();

	int exec();
	void exit(int code = 0);

	void callLater(const std::function<void()> &func);

private:
	static EventLoop *instance_;

	struct event_base *base_;
	int exitCode_;

	std::list<std::function<void()>> calls_;
	std::mutex lock_;

	static void dispatchCallback(evutil_socket_t fd, short flags,
				     void *param);
	void dispatchCall();
};

#endif /* __CAM_EVENT_LOOP_H__ */
