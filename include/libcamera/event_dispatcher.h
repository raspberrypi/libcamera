/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_dispatcher.h - Event dispatcher
 */
#ifndef __LIBCAMERA_EVENT_DISPATCHER_H__
#define __LIBCAMERA_EVENT_DISPATCHER_H__

#include <vector>

namespace libcamera {

class EventNotifier;
class Timer;

class EventDispatcher
{
public:
	virtual ~EventDispatcher();

	virtual void registerEventNotifier(EventNotifier *notifier) = 0;
	virtual void unregisterEventNotifier(EventNotifier *notifier) = 0;

	virtual void registerTimer(Timer *timer) = 0;
	virtual void unregisterTimer(Timer *timer) = 0;

	virtual void processEvents() = 0;

	virtual void interrupt() = 0;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_EVENT_DISPATCHER_H__ */
