/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_loop.h - cam - Event loop
 */
#ifndef __CAM_EVENT_LOOP_H__
#define __CAM_EVENT_LOOP_H__

#include <atomic>

#include <libcamera/event_notifier.h>

namespace libcamera {
class EventDispatcher;
};

class EventLoop
{
public:
	EventLoop(libcamera::EventDispatcher *dispatcher);
	~EventLoop();

	int exec();
	void exit(int code = 0);

private:
	libcamera::EventDispatcher *dispatcher_;

	std::atomic<bool> exit_;
	int exitCode_;
};

#endif /* __CAM_EVENT_LOOP_H__ */
