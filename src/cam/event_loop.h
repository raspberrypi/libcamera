/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_loop.h - cam - Event loop
 */
#ifndef __CAM_EVENT_LOOP_H__
#define __CAM_EVENT_LOOP_H__

#include <atomic>

struct event_base;

class EventLoop
{
public:
	EventLoop();
	~EventLoop();

	static EventLoop *instance();

	int exec();
	void exit(int code = 0);

private:
	static EventLoop *instance_;

	struct event_base *event_;
	std::atomic<bool> exit_;
	int exitCode_;

	void interrupt();
};

#endif /* __CAM_EVENT_LOOP_H__ */
