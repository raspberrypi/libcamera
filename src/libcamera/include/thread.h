/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread.h - Thread support
 */
#ifndef __LIBCAMERA_THREAD_H__
#define __LIBCAMERA_THREAD_H__

#include <memory>
#include <mutex>
#include <thread>

#include <libcamera/signal.h>

namespace libcamera {

class EventDispatcher;
class Message;
class Object;
class ThreadData;
class ThreadMain;

using Mutex = std::mutex;
using MutexLocker = std::unique_lock<std::mutex>;

class Thread
{
public:
	Thread();
	virtual ~Thread();

	void start();
	void exit(int code = 0);
	void wait();

	bool isRunning();

	Signal<Thread *> finished;

	static Thread *current();

	EventDispatcher *eventDispatcher();
	void setEventDispatcher(std::unique_ptr<EventDispatcher> dispatcher);

	void dispatchMessages();

protected:
	int exec();
	virtual void run();

private:
	void startThread();
	void finishThread();

	void postMessage(std::unique_ptr<Message> msg, Object *receiver);
	void removeMessages(Object *receiver);

	friend class Object;
	friend class ThreadData;
	friend class ThreadMain;

	void moveObject(Object *object);

	std::thread thread_;
	ThreadData *data_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_THREAD_H__ */
