/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * File descriptor event notifier
 */

#pragma once

#include <libcamera/base/private.h>

#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>

namespace libcamera {

class Message;

class EventNotifier : public Object
{
public:
	enum Type {
		Read,
		Write,
		Exception,
	};

	EventNotifier(int fd, Type type, Object *parent = nullptr);
	virtual ~EventNotifier();

	Type type() const { return type_; }
	int fd() const { return fd_; }

	bool enabled() const { return enabled_; }
	void setEnabled(bool enable);

	Signal<> activated;

protected:
	void message(Message *msg) override;

private:
	int fd_;
	Type type_;
	bool enabled_;
};

} /* namespace libcamera */
