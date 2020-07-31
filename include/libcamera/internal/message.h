/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * message.h - Message queue support
 */
#ifndef __LIBCAMERA_INTERNAL_MESSAGE_H__
#define __LIBCAMERA_INTERNAL_MESSAGE_H__

#include <atomic>

#include <libcamera/bound_method.h>

namespace libcamera {

class BoundMethodBase;
class Object;
class Semaphore;
class Thread;

class Message
{
public:
	enum Type {
		None = 0,
		InvokeMessage = 1,
		ThreadMoveMessage = 2,
		DeferredDelete = 3,
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
	InvokeMessage(BoundMethodBase *method,
		      std::shared_ptr<BoundMethodPackBase> pack,
		      Semaphore *semaphore = nullptr,
		      bool deleteMethod = false);
	~InvokeMessage();

	Semaphore *semaphore() const { return semaphore_; }

	void invoke();

private:
	BoundMethodBase *method_;
	std::shared_ptr<BoundMethodPackBase> pack_;
	Semaphore *semaphore_;
	bool deleteMethod_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_MESSAGE_H__ */
