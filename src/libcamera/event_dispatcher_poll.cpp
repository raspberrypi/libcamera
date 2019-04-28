/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_dispatcher_poll.cpp - Poll-based event dispatcher
 */

#include "event_dispatcher_poll.h"

#include <algorithm>
#include <iomanip>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <libcamera/event_notifier.h>
#include <libcamera/timer.h>

#include "log.h"

/**
 * \file event_dispatcher_poll.h
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(Event)

static const char *notifierType(EventNotifier::Type type)
{
	if (type == EventNotifier::Read)
		return "read";
	if (type == EventNotifier::Write)
		return "write";
	if (type == EventNotifier::Exception)
		return "exception";

	return "";
}

/**
 * \class EventDispatcherPoll
 * \brief A poll-based event dispatcher
 */

EventDispatcherPoll::EventDispatcherPoll()
{
	/*
	 * Create the event fd. Failures are fatal as we can't implement an
	 * interruptible dispatcher without the fd.
	 */
	eventfd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (eventfd_ < 0)
		LOG(Event, Fatal) << "Unable to create eventfd";
}

EventDispatcherPoll::~EventDispatcherPoll()
{
	close(eventfd_);
}

void EventDispatcherPoll::registerEventNotifier(EventNotifier *notifier)
{
	EventNotifierSetPoll &set = notifiers_[notifier->fd()];
	EventNotifier::Type type = notifier->type();

	if (set.notifiers[type] && set.notifiers[type] != notifier) {
		LOG(Event, Warning)
			<< "Ignoring duplicate " << notifierType(type)
			<< " notifier for fd " << notifier->fd();
		return;
	}

	set.notifiers[type] = notifier;
}

void EventDispatcherPoll::unregisterEventNotifier(EventNotifier *notifier)
{
	auto iter = notifiers_.find(notifier->fd());
	if (iter == notifiers_.end())
		return;

	EventNotifierSetPoll &set = iter->second;
	EventNotifier::Type type = notifier->type();

	if (!set.notifiers[type])
		return;

	if (set.notifiers[type] != notifier) {
		LOG(Event, Warning)
			<< notifierType(type) << " notifier for fd "
			<< notifier->fd() << " is not registered";
		return;
	}

	set.notifiers[type] = nullptr;
	if (!set.notifiers[0] && !set.notifiers[1] && !set.notifiers[2])
		notifiers_.erase(iter);
}

void EventDispatcherPoll::registerTimer(Timer *timer)
{
	for (auto iter = timers_.begin(); iter != timers_.end(); ++iter) {
		if ((*iter)->deadline() > timer->deadline()) {
			timers_.insert(iter, timer);
			return;
		}
	}

	timers_.push_back(timer);
}

void EventDispatcherPoll::unregisterTimer(Timer *timer)
{
	for (auto iter = timers_.begin(); iter != timers_.end(); ++iter) {
		if (*iter == timer) {
			timers_.erase(iter);
			return;
		}

		/*
		 * As the timers list is ordered, we can stop as soon as we go
		 * past the deadline.
		 */
		if ((*iter)->deadline() > timer->deadline())
			break;
	}
}

void EventDispatcherPoll::processEvents()
{
	int ret;

	/* Create the pollfd array. */
	std::vector<struct pollfd> pollfds;
	pollfds.reserve(notifiers_.size() + 1);

	for (auto notifier : notifiers_)
		pollfds.push_back({ notifier.first, notifier.second.events(), 0 });

	pollfds.push_back({ eventfd_, POLLIN, 0 });

	/* Wait for events and process notifiers and timers. */
	do {
		ret = poll(&pollfds);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		ret = -errno;
		LOG(Event, Warning) << "poll() failed with " << strerror(-ret);
	} else if (ret > 0) {
		processInterrupt(pollfds.back());
		pollfds.pop_back();
		processNotifiers(pollfds);
	}

	processTimers();
}

void EventDispatcherPoll::interrupt()
{
	uint64_t value = 1;
	ssize_t ret = write(eventfd_, &value, sizeof(value));
	if (ret != sizeof(value)) {
		if (ret < 0)
			ret = -errno;
		LOG(Event, Error)
			<< "Failed to interrupt event dispatcher ("
			<< ret << ")";
	}
}

short EventDispatcherPoll::EventNotifierSetPoll::events() const
{
	short events = 0;

	if (notifiers[EventNotifier::Read])
		events |= POLLIN;
	if (notifiers[EventNotifier::Write])
		events |= POLLOUT;
	if (notifiers[EventNotifier::Exception])
		events |= POLLPRI;

	return events;
}

int EventDispatcherPoll::poll(std::vector<struct pollfd> *pollfds)
{
	/* Compute the timeout. */
	Timer *nextTimer = !timers_.empty() ? timers_.front() : nullptr;
	struct timespec timeout;

	if (nextTimer) {
		clock_gettime(CLOCK_MONOTONIC, &timeout);
		uint64_t now = timeout.tv_sec * 1000000000ULL + timeout.tv_nsec;

		if (nextTimer->deadline() > now) {
			uint64_t delta = nextTimer->deadline() - now;
			timeout.tv_sec = delta / 1000000000ULL;
			timeout.tv_nsec = delta % 1000000000ULL;
		} else {
			timeout.tv_sec = 0;
			timeout.tv_nsec = 0;
		}

		LOG(Event, Debug)
			<< "timeout " << timeout.tv_sec << "."
			<< std::setfill('0') << std::setw(9)
			<< timeout.tv_nsec;
	}

	return ppoll(pollfds->data(), pollfds->size(),
		     nextTimer ? &timeout : nullptr, nullptr);
}

void EventDispatcherPoll::processInterrupt(const struct pollfd &pfd)
{
	if (!(pfd.revents & POLLIN))
		return;

	uint64_t value;
	ssize_t ret = read(eventfd_, &value, sizeof(value));
	if (ret != sizeof(value)) {
		if (ret < 0)
			ret = -errno;
		LOG(Event, Error)
			<< "Failed to process interrupt (" << ret << ")";
	}
}

void EventDispatcherPoll::processNotifiers(const std::vector<struct pollfd> &pollfds)
{
	static const struct {
		EventNotifier::Type type;
		short events;
	} events[] = {
		{ EventNotifier::Read, POLLIN },
		{ EventNotifier::Write, POLLOUT },
		{ EventNotifier::Exception, POLLPRI },
	};

	for (const struct pollfd &pfd : pollfds) {
		auto iter = notifiers_.find(pfd.fd);
		ASSERT(iter != notifiers_.end());

		EventNotifierSetPoll &set = iter->second;

		for (const auto &event : events) {
			EventNotifier *notifier = set.notifiers[event.type];

			if (!notifier)
				continue;

			/*
			 * If the file descriptor is invalid, disable the
			 * notifier immediately.
			 */
			if (pfd.revents & POLLNVAL) {
				LOG(Event, Warning)
					<< "Disabling " << notifierType(event.type)
					<< " due to invalid file descriptor "
					<< pfd.fd;
				unregisterEventNotifier(notifier);
				continue;
			}

			if (pfd.revents & event.events)
				notifier->activated.emit(notifier);
		}
	}
}

void EventDispatcherPoll::processTimers()
{
	struct timespec ts;
	uint64_t now;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	now = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

	while (!timers_.empty()) {
		Timer *timer = timers_.front();
		if (timer->deadline() > now)
			break;

		timers_.pop_front();
		timer->stop();
		timer->timeout.emit(timer);
	}
}

} /* namespace libcamera */
