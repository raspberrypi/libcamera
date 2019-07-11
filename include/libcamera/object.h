/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * object.h - Base object
 */
#ifndef __LIBCAMERA_OBJECT_H__
#define __LIBCAMERA_OBJECT_H__

#include <list>
#include <memory>

namespace libcamera {

class Message;
template<typename... Args>
class Signal;
class SignalBase;
class SlotBase;
class Thread;

class Object
{
public:
	Object();
	virtual ~Object();

	void postMessage(std::unique_ptr<Message> msg);
	virtual void message(Message *msg);

	Thread *thread() const { return thread_; }
	void moveToThread(Thread *thread);

private:
	template<typename... Args>
	friend class Signal;
	friend class SlotBase;
	friend class Thread;

	void connect(SignalBase *signal);
	void disconnect(SignalBase *signal);

	Thread *thread_;
	std::list<SignalBase *> signals_;
	unsigned int pendingMessages_;
};

}; /* namespace libcamera */

#endif /* __LIBCAMERA_OBJECT_H__ */
