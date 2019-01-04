/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * event_notifier.h - File descriptor event notifier
 */
#ifndef __LIBCAMERA_EVENT_NOTIFIER_H__
#define __LIBCAMERA_EVENT_NOTIFIER_H__

#include <libcamera/signal.h>

namespace libcamera {

class EventNotifier
{
public:
	enum Type {
		Read,
		Write,
		Exception,
	};

	EventNotifier(int fd, Type type);
	virtual ~EventNotifier();

	Type type() const { return type_; }
	int fd() const { return fd_; }

	bool enabled() const { return enabled_; }
	void setEnabled(bool enable);

	Signal<EventNotifier *> activated;

private:
	int fd_;
	Type type_;
	bool enabled_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_EVENT_NOTIFIER_H__ */
