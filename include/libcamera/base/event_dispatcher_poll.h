/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Poll-based event dispatcher
 */

#pragma once

#include <list>
#include <map>
#include <vector>

#include <libcamera/base/private.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/unique_fd.h>

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

	int poll(std::vector<struct pollfd> *pollfds);
	void processInterrupt(const struct pollfd &pfd);
	void processNotifiers(const std::vector<struct pollfd> &pollfds);
	void processTimers();

	std::map<int, EventNotifierSetPoll> notifiers_;
	std::list<Timer *> timers_;
	UniqueFD eventfd_;

	bool processingEvents_;
};

} /* namespace libcamera */
