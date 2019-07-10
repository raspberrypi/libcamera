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
class SlotBase;
class Thread;

class Message
{
public:
	enum Type {
		None = 0,
		SignalMessage = 1,
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

class SignalMessage : public Message
{
public:
	SignalMessage(SlotBase *slot, void *pack)
		: Message(Message::SignalMessage), slot_(slot), pack_(pack)
	{
	}

	SlotBase *slot_;
	void *pack_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_MESSAGE_H__ */
