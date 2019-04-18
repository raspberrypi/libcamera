/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_dispatcher_poll.h - Poll-based event dispatcher
 */
#ifndef __LIBCAMERA_EVENT_DISPATCHER_POLL_H__
#define __LIBCAMERA_EVENT_DISPATCHER_POLL_H__

#include <libcamera/event_dispatcher.h>

#include <list>
#include <map>
#include <vector>

struct pollfd;

namespace libcamera {

class EventNotifier;
class Timer;

class EventDispatcherPoll final : public EventDispatcher
{
public:
	EventDispatcherPoll();
	~EventDispatcherPoll();

	void registerEventNotifier(EventNotifier *notifier);
	void unregisterEventNotifier(EventNotifier *notifier);

	void registerTimer(Timer *timer);
	void unregisterTimer(Timer *timer);

	void processEvents();
	void interrupt();

private:
	struct EventNotifierSetPoll {
		short events() const;
		EventNotifier *notifiers[3];
	};

	std::map<int, EventNotifierSetPoll> notifiers_;
	std::list<Timer *> timers_;
	int eventfd_;

	int poll(std::vector<struct pollfd> *pollfds);
	void processInterrupt(const struct pollfd &pfd);
	void processNotifiers(const std::vector<struct pollfd> &pollfds);
	void processTimers();
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_EVENT_DISPATCHER_POLL_H__ */
