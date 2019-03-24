/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * message.h - Message queue support
 */
#ifndef __LIBCAMERA_MESSAGE_H__
#define __LIBCAMERA_MESSAGE_H__

namespace libcamera {

class Object;
class Thread;

class Message
{
public:
	enum Type {
		None = 0,
	};

	Message(Type type);
	virtual ~Message();

	Type type() const { return type_; }
	Object *receiver() const { return receiver_; }

private:
	friend class Thread;

	Type type_;
	Object *receiver_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_MESSAGE_H__ */
