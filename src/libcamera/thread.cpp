/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread.cpp - Thread support
 */

#include "libcamera/internal/thread.h"

#include <atomic>
#include <condition_variable>
#include <list>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include "libcamera/internal/event_dispatcher.h"
#include "libcamera/internal/event_dispatcher_poll.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/message.h"

/**
 * \page thread Thread Support
 *
 * libcamera supports multi-threaded applications through a threading model that
 * sets precise rules to guarantee thread-safe usage of the API. Additionally,
 * libcamera makes internal use of threads, and offers APIs that simplify
 * interactions with application threads. Careful compliance with the threading
 * model will ensure avoidance of race conditions.
 *
 * Every thread created by libcamera is associated with an instance of the
 * Thread class. Those threads run an internal event loop by default to
 * dispatch events to objects. Additionally, the main thread of the application
 * (defined as the thread that calls CameraManager::start()) is also associated
 * with a Thread instance, but has no event loop accessible to libcamera. Other
 * application threads are not visible to libcamera.
 *
 * \section thread-objects Threads and Objects
 *
 * Instances of the Object class and all its derived classes are thread-aware
 * and are bound to the thread they are created in. They are said to *live* in
 * a thread, and they interact with the event loop of their thread for the
 * purpose of message passing and signal delivery. Messages posted to the
 * object with Object::postMessage() will be delivered from the event loop of
 * the thread that the object lives in. Signals delivered to the object, unless
 * explicitly connected with ConnectionTypeDirect, will also be delivered from
 * the object thread's event loop.
 *
 * All Object instances created internally by libcamera are bound to internal
 * threads. As objects interact with thread event loops for proper operation,
 * creating an Object instance in a thread that has no internal event loop (such
 * as the main application thread, or libcamera threads that have a custom main
 * loop), prevents some features of the Object class from being used. See
 * Thread::exec() for more details.
 *
 * \section thread-signals Threads and Signals
 *
 * When sent to a receiver that does not inherit from the Object class, signals
 * are delivered synchronously in the thread of the sender. When the receiver
 * inherits from the Object class, delivery is by default asynchronous if the
 * sender and receiver live in different threads. In that case, the signal is
 * posted to the receiver's message queue and will be delivered from the
 * receiver's event loop, running in the receiver's thread. This mechanism can
 * be overridden by selecting a different connection type when calling
 * Signal::connect().
 *
 * Asynchronous signal delivery is used internally in libcamera, but is also
 * available to applications if desired. To use this feature, applications
 * shall create receiver classes that inherit from the Object class, and
 * provide an event loop to the CameraManager as explained above. Note that
 * Object instances created by the application are limited to living in the
 * application's main thread. Creating Object instances from another thread of
 * an application causes undefined behaviour.
 *
 * \section thread-reentrancy Reentrancy and Thread-Safety
 *
 * Through the documentation, several terms are used to define how classes and
 * their member functions can be used from multiple threads.
 *
 * - A **reentrant** function may be called simultaneously from multiple
 *   threads if and only if each invocation uses a different instance of the
 *   class. This is the default for all member functions not explictly marked
 *   otherwise.
 *
 * - \anchor thread-safe A **thread-safe** function may be called
 *   simultaneously from multiple threads on the same instance of a class. A
 *   thread-safe function is thus reentrant. Thread-safe functions may also be
 *   called simultaneously with any other reentrant function of the same class
 *   on the same instance.
 *
 * - \anchor thread-bound A **thread-bound** function may be called only from
 *   the thread that the class instances lives in (see section \ref
 *   thread-objects). For instances of classes that do not derive from the
 *   Object class, this is the thread in which the instance was created. A
 *   thread-bound function is not thread-safe, and may or may not be reentrant.
 *
 * Neither reentrancy nor thread-safety, in this context, mean that a function
 * may be called simultaneously from the same thread, for instance from a
 * callback invoked by the function. This may deadlock and isn't allowed unless
 * separately documented.
 *
 * A class is defined as reentrant, thread-safe or thread-bound if all its
 * member functions are reentrant, thread-safe or thread-bound respectively.
 * Some member functions may additionally be documented as having additional
 * thread-related attributes.
 *
 * Most classes are reentrant but not thread-safe, as making them fully
 * thread-safe would incur locking costs considered prohibitive for the
 * expected use cases.
 */

/**
 * \file thread.h
 * \brief Thread support
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Thread)

class ThreadMain;

/**
 * \brief A queue of posted messages
 */
class MessageQueue
{
public:
	/**
	 * \brief List of queued Message instances
	 */
	std::list<std::unique_ptr<Message>> list_;
	/**
	 * \brief Protects the \ref list_
	 */
	Mutex mutex_;
};

/**
 * \brief Thread-local internal data
 */
class ThreadData
{
public:
	ThreadData()
		: thread_(nullptr), running_(false), dispatcher_(nullptr)
	{
	}

	static ThreadData *current();

private:
	friend class Thread;
	friend class ThreadMain;

	Thread *thread_;
	bool running_;
	pid_t tid_;

	Mutex mutex_;

	std::atomic<EventDispatcher *> dispatcher_;

	std::condition_variable cv_;
	std::atomic<bool> exit_;
	int exitCode_;

	MessageQueue messages_;
};

/**
 * \brief Thread wrapper for the main thread
 */
class ThreadMain : public Thread
{
public:
	ThreadMain()
	{
		data_->running_ = true;
	}

protected:
	void run() override
	{
		LOG(Thread, Fatal) << "The main thread can't be restarted";
	}
};

static thread_local ThreadData *currentThreadData = nullptr;
static ThreadMain mainThread;

/**
 * \brief Retrieve thread-local internal data for the current thread
 * \return The thread-local internal data for the current thread
 */
ThreadData *ThreadData::current()
{
	if (currentThreadData)
		return currentThreadData;

	/*
	 * The main thread doesn't receive thread-local data when it is
	 * started, set it here.
	 */
	ThreadData *data = mainThread.data_;
	data->tid_ = syscall(SYS_gettid);
	currentThreadData = data;
	return data;
}

/**
 * \typedef Mutex
 * \brief An alias for std::mutex
 */

/**
 * \typedef MutexLocker
 * \brief An alias for std::unique_lock<std::mutex>
 */

/**
 * \class Thread
 * \brief A thread of execution
 *
 * The Thread class is a wrapper around std::thread that handles integration
 * with the Object, Signal and EventDispatcher classes.
 *
 * Thread instances by default run an event loop until the exit() method is
 * called. The event loop dispatches events (messages, notifiers and timers)
 * sent to the objects living in the thread. This behaviour can be modified by
 * overriding the run() function.
 */

/**
 * \brief Create a thread
 */
Thread::Thread()
{
	data_ = new ThreadData;
	data_->thread_ = this;
}

Thread::~Thread()
{
	delete data_->dispatcher_.load(std::memory_order_relaxed);
	delete data_;
}

/**
 * \brief Start the thread
 */
void Thread::start()
{
	MutexLocker locker(data_->mutex_);

	if (data_->running_)
		return;

	data_->running_ = true;
	data_->exitCode_ = -1;
	data_->exit_.store(false, std::memory_order_relaxed);

	thread_ = std::thread(&Thread::startThread, this);
}

void Thread::startThread()
{
	struct ThreadCleaner {
		ThreadCleaner(Thread *thread, void (Thread::*cleaner)())
			: thread_(thread), cleaner_(cleaner)
		{
		}
		~ThreadCleaner()
		{
			(thread_->*cleaner_)();
		}

		Thread *thread_;
		void (Thread::*cleaner_)();
	};

	/*
	 * Make sure the thread is cleaned up even if the run method exits
	 * abnormally (for instance via a direct call to pthread_cancel()).
	 */
	thread_local ThreadCleaner cleaner(this, &Thread::finishThread);

	data_->tid_ = syscall(SYS_gettid);
	currentThreadData = data_;

	run();
}

/**
 * \brief Enter the event loop
 *
 * This method enters an event loop based on the event dispatcher instance for
 * the thread, and blocks until the exit() method is called. It is meant to be
 * called within the thread from the run() method and shall not be called
 * outside of the thread.
 *
 * \return The exit code passed to the exit() method
 */
int Thread::exec()
{
	MutexLocker locker(data_->mutex_);

	EventDispatcher *dispatcher = eventDispatcher();

	locker.unlock();

	while (!data_->exit_.load(std::memory_order_acquire))
		dispatcher->processEvents();

	locker.lock();

	return data_->exitCode_;
}

/**
 * \brief Main method of the thread
 *
 * When the thread is started with start(), it calls this method in the context
 * of the new thread. The run() method can be overridden to perform custom
 * work, either custom initialization and cleanup before and after calling the
 * Thread::exec() function, or a custom thread loop altogether. When this
 * method returns the thread execution is stopped, and the \ref finished signal
 * is emitted.
 *
 * Note that if this function is overridden and doesn't call Thread::exec(), no
 * events will be dispatched to the objects living in the thread. These objects
 * will not be able to use the EventNotifier, Timer or Message facilities. This
 * includes functions that rely on message dispatching, such as
 * Object::deleteLater().
 *
 * The base implementation just calls exec().
 */
void Thread::run()
{
	exec();
}

void Thread::finishThread()
{
	data_->mutex_.lock();
	data_->running_ = false;
	data_->mutex_.unlock();

	finished.emit(this);
	data_->cv_.notify_all();
}

/**
 * \brief Stop the thread's event loop
 * \param[in] code The exit code
 *
 * This method interrupts the event loop started by the exec() method, causing
 * exec() to return \a code.
 *
 * Calling exit() on a thread that reimplements the run() method and doesn't
 * call exec() will likely have no effect.
 *
 * \context This function is \threadsafe.
 */
void Thread::exit(int code)
{
	data_->exitCode_ = code;
	data_->exit_.store(true, std::memory_order_release);

	EventDispatcher *dispatcher = data_->dispatcher_.load(std::memory_order_relaxed);
	if (!dispatcher)
		return;

	dispatcher->interrupt();
}

/**
 * \brief Wait for the thread to finish
 * \param[in] duration Maximum wait duration
 *
 * This function waits until the thread finishes or the \a duration has
 * elapsed, whichever happens first. If \a duration is equal to
 * utils::duration::max(), the wait never times out. If the thread is not
 * running the function returns immediately.
 *
 * \context This function is \threadsafe.
 *
 * \return True if the thread has finished, or false if the wait timed out
 */
bool Thread::wait(utils::duration duration)
{
	bool hasFinished = true;

	{
		MutexLocker locker(data_->mutex_);

		if (duration == utils::duration::max())
			data_->cv_.wait(locker, [&]() { return !data_->running_; });
		else
			hasFinished = data_->cv_.wait_for(locker, duration,
							  [&]() { return !data_->running_; });
	}

	if (thread_.joinable())
		thread_.join();

	return hasFinished;
}

/**
 * \brief Check if the thread is running
 *
 * A Thread instance is considered as running once the underlying thread has
 * started. This method guarantees that it returns true after the start()
 * method returns, and false after the wait() method returns.
 *
 * \context This function is \threadsafe.
 *
 * \return True if the thread is running, false otherwise
 */
bool Thread::isRunning()
{
	MutexLocker locker(data_->mutex_);
	return data_->running_;
}

/**
 * \var Thread::finished
 * \brief Signal the end of thread execution
 */

/**
 * \brief Retrieve the Thread instance for the current thread
 * \context This function is \threadsafe.
 * \return The Thread instance for the current thread
 */
Thread *Thread::current()
{
	ThreadData *data = ThreadData::current();
	return data->thread_;
}

/**
 * \brief Retrieve the ID of the current thread
 *
 * The thread ID corresponds to the Linux thread ID (TID) as returned by the
 * gettid system call.
 *
 * \context This function is \threadsafe.
 *
 * \return The ID of the current thread
 */
pid_t Thread::currentId()
{
	ThreadData *data = ThreadData::current();
	return data->tid_;
}

/**
 * \brief Retrieve the event dispatcher
 *
 * This function retrieves the internal event dispatcher for the thread. The
 * returned event dispatcher is valid until the thread is destroyed.
 *
 * \context This function is \threadsafe.
 *
 * \return Pointer to the event dispatcher
 */
EventDispatcher *Thread::eventDispatcher()
{
	if (!data_->dispatcher_.load(std::memory_order_relaxed))
		data_->dispatcher_.store(new EventDispatcherPoll(),
					 std::memory_order_release);

	return data_->dispatcher_.load(std::memory_order_relaxed);
}

/**
 * \brief Post a message to the thread for the \a receiver
 * \param[in] msg The message
 * \param[in] receiver The receiver
 *
 * This method stores the message \a msg in the message queue of the thread for
 * the \a receiver and wake up the thread's event loop. Message ownership is
 * passed to the thread, and the message will be deleted after being delivered.
 *
 * Messages are delivered through the thread's event loop. If the thread is not
 * running its event loop the message will not be delivered until the event
 * loop gets started.
 *
 * If the \a receiver is not bound to this thread the behaviour is undefined.
 *
 * \sa exec()
 */
void Thread::postMessage(std::unique_ptr<Message> msg, Object *receiver)
{
	msg->receiver_ = receiver;

	ASSERT(data_ == receiver->thread()->data_);

	MutexLocker locker(data_->messages_.mutex_);
	data_->messages_.list_.push_back(std::move(msg));
	receiver->pendingMessages_++;
	locker.unlock();

	EventDispatcher *dispatcher =
		data_->dispatcher_.load(std::memory_order_acquire);
	if (dispatcher)
		dispatcher->interrupt();
}

/**
 * \brief Remove all posted messages for the \a receiver
 * \param[in] receiver The receiver
 *
 * If the \a receiver is not bound to this thread the behaviour is undefined.
 */
void Thread::removeMessages(Object *receiver)
{
	ASSERT(data_ == receiver->thread()->data_);

	MutexLocker locker(data_->messages_.mutex_);
	if (!receiver->pendingMessages_)
		return;

	std::vector<std::unique_ptr<Message>> toDelete;
	for (std::unique_ptr<Message> &msg : data_->messages_.list_) {
		if (!msg)
			continue;
		if (msg->receiver_ != receiver)
			continue;

		/*
		 * Move the message to the pending deletion list to delete it
		 * after releasing the lock. The messages list element will
		 * contain a null pointer, and will be removed when dispatching
		 * messages.
		 */
		toDelete.push_back(std::move(msg));
		receiver->pendingMessages_--;
	}

	ASSERT(!receiver->pendingMessages_);
	locker.unlock();

	toDelete.clear();
}

/**
 * \brief Dispatch posted messages for this thread
 * \param[in] type The message type
 *
 * This function immediately dispatches all the messages previously posted for
 * this thread with postMessage() that match the message \a type. If the \a type
 * is Message::Type::None, all messages are dispatched.
 *
 * Messages shall only be dispatched from the current thread, typically within
 * the thread from the run() function. Calling this function outside of the
 * thread results in undefined behaviour.
 */
void Thread::dispatchMessages(Message::Type type)
{
	ASSERT(data_ == ThreadData::current());

	MutexLocker locker(data_->messages_.mutex_);

	std::list<std::unique_ptr<Message>> &messages = data_->messages_.list_;

	for (auto iter = messages.begin(); iter != messages.end(); ) {
		std::unique_ptr<Message> &msg = *iter;

		if (!msg) {
			iter = data_->messages_.list_.erase(iter);
			continue;
		}

		if (type != Message::Type::None && msg->type() != type) {
			++iter;
			continue;
		}

		std::unique_ptr<Message> message = std::move(msg);
		iter = data_->messages_.list_.erase(iter);

		Object *receiver = message->receiver_;
		ASSERT(data_ == receiver->thread()->data_);
		receiver->pendingMessages_--;

		locker.unlock();
		receiver->message(message.get());
		message.reset();
		locker.lock();
	}
}

/**
 * \brief Move an \a object and all its children to the thread
 * \param[in] object The object
 */
void Thread::moveObject(Object *object)
{
	ThreadData *currentData = object->thread_->data_;
	ThreadData *targetData = data_;

	MutexLocker lockerFrom(currentData->messages_.mutex_, std::defer_lock);
	MutexLocker lockerTo(targetData->messages_.mutex_, std::defer_lock);
	std::lock(lockerFrom, lockerTo);

	moveObject(object, currentData, targetData);
}

void Thread::moveObject(Object *object, ThreadData *currentData,
			ThreadData *targetData)
{
	/* Move pending messages to the message queue of the new thread. */
	if (object->pendingMessages_) {
		unsigned int movedMessages = 0;

		for (std::unique_ptr<Message> &msg : currentData->messages_.list_) {
			if (!msg)
				continue;
			if (msg->receiver_ != object)
				continue;

			targetData->messages_.list_.push_back(std::move(msg));
			movedMessages++;
		}

		if (movedMessages) {
			EventDispatcher *dispatcher =
				targetData->dispatcher_.load(std::memory_order_acquire);
			if (dispatcher)
				dispatcher->interrupt();
		}
	}

	object->thread_ = this;

	/* Move all children. */
	for (auto child : object->children_)
		moveObject(child, currentData, targetData);
}

} /* namespace libcamera */
