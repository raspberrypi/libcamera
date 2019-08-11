/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * message.h - Message queue support
 */
#ifndef __LIBCAMERA_MESSAGE_H__
#define __LIBCAMERA_MESSAGE_H__

#include <atomic>

#include <libcamera/bound_method.h>

namespace libcamera {

class BoundMethodBase;
class Object;
class Thread;

class Message
{
public:
	enum Type {
		None = 0,
		InvokeMessage = 1,
		UserMessage = 1000,
	};

	Message(Type type);
	virtual ~Message();

	Type type() const { return type_; }
	Object *receiver() const { return receiver_; }

	static Type registerMessageType();

private:
	friend class Thread;

	Type type_;
	Object *receiver_;

	static std::atomic_uint nextUserType_;
};

class InvokeMessage : public Message
{
public:
	InvokeMessage(BoundMethodBase *method, void *pack,
		      bool deleteMethod = false);
	~InvokeMessage();

	void invoke();

private:
	BoundMethodBase *method_;
	void *pack_;
	bool deleteMethod_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_MESSAGE_H__ */
