/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread.h - Thread support
 */
#ifndef __LIBCAMERA_INTERNAL_THREAD_H__
#define __LIBCAMERA_INTERNAL_THREAD_H__

#include <memory>
#include <mutex>
#include <sys/types.h>
#include <thread>

#include <libcamera/signal.h>

#include "libcamera/internal/message.h"
#include "libcamera/internal/utils.h"

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
	bool wait(utils::duration duration = utils::duration::max());

	bool isRunning();

	Signal<Thread *> finished;

	static Thread *current();
	static pid_t currentId();

	EventDispatcher *eventDispatcher();
	void setEventDispatcher(std::unique_ptr<EventDispatcher> dispatcher);

	void dispatchMessages(Message::Type type = Message::Type::None);

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
	void moveObject(Object *object, ThreadData *currentData,
			ThreadData *targetData);

	std::thread thread_;
	ThreadData *data_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_THREAD_H__ */
