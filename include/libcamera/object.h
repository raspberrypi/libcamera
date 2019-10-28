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
#include <vector>

#include <libcamera/bound_method.h>

namespace libcamera {

class Message;
template<typename... Args>
class Signal;
class SignalBase;
class Thread;

class Object
{
public:
	Object(Object *parent = nullptr);
	virtual ~Object();

	void postMessage(std::unique_ptr<Message> msg);

	template<typename T, typename... Args, typename std::enable_if<std::is_base_of<Object, T>::value>::type * = nullptr>
	void invokeMethod(void (T::*func)(Args...), Args... args)
	{
		T *obj = static_cast<T *>(this);
		BoundMethodBase *method =
			new BoundMemberMethod<T, Args...>(obj, this, func,
							  ConnectionTypeQueued);
		void *pack = new typename BoundMemberMethod<T, Args...>::PackType{ args... };

		method->activatePack(pack, true);
	}

	Thread *thread() const { return thread_; }
	void moveToThread(Thread *thread);

	Object *parent() const { return parent_; }

protected:
	virtual void message(Message *msg);

private:
	template<typename... Args>
	friend class Signal;
	friend class BoundMethodBase;
	friend class Thread;

	void notifyThreadMove();

	void connect(SignalBase *signal);
	void disconnect(SignalBase *signal);

	Object *parent_;
	std::vector<Object *> children_;

	Thread *thread_;
	std::list<SignalBase *> signals_;
	unsigned int pendingMessages_;
};

}; /* namespace libcamera */

#endif /* __LIBCAMERA_OBJECT_H__ */
