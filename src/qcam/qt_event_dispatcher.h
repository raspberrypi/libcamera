/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * qt_event_dispatcher.h - qcam - Qt-based event dispatcher
 */
#ifndef __QCAM_QT_EVENT_DISPATCHER_H__
#define __QCAM_QT_EVENT_DISPATCHER_H__

#include <map>

#include <libcamera/event_dispatcher.h>

using namespace libcamera;

class QSocketNotifier;

class QtEventDispatcher final : public EventDispatcher, public QObject
{
public:
	QtEventDispatcher();
	~QtEventDispatcher();

	void registerEventNotifier(EventNotifier *notifier);
	void unregisterEventNotifier(EventNotifier *notifier);

	void registerTimer(Timer *timer);
	void unregisterTimer(Timer *timer);

	void processEvents();

	void interrupt();

protected:
	void timerEvent(QTimerEvent *event);

private:
	void readNotifierActivated(int socket);
	void writeNotifierActivated(int socket);
	void exceptionNotifierActivated(int socket);

	struct NotifierPair {
		NotifierPair()
			: notifier(nullptr), qnotifier(nullptr)
		{
		}
		EventNotifier *notifier;
		QSocketNotifier *qnotifier;
	};

	struct NotifierSet {
		NotifierPair read;
		NotifierPair write;
		NotifierPair exception;
	};

	std::map<int, NotifierSet> notifiers_;
	std::map<int, Timer *> timers_;
	std::map<Timer *, int> timerIds_;
};

#endif /* __QCAM_QT_EVENT_DISPATCHER_H__ */
