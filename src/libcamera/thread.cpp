/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * thread.cpp - Thread support
 */

#include "thread.h"

#include <atomic>

#include <libcamera/event_dispatcher.h>

#include "event_dispatcher_poll.h"
#include "log.h"

/**
 * \file thread.h
 * \brief Thread support
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Thread)

class ThreadMain;

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

	Mutex mutex_;

	std::atomic<EventDispatcher *> dispatcher_;

	std::atomic<bool> exit_;
	int exitCode_;
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
 * called. A custom event dispatcher may be installed with
 * setEventDispatcher(), otherwise a poll-based event dispatcher is used. This
 * behaviour can be overriden by overloading the run() method.
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

	currentThreadData = data_;

	run();
}

/**
 * \brief Enter the event loop
 *
 * This method enter an event loop based on the event dispatcher instance for
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
 * of the new thread. The run() method can be overloaded to perform custom
 * work. When this method returns the thread execution is stopped, and the \ref
 * finished signal is emitted.
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
 *
 * This method waits until the thread finishes, or returns immediately if the
 * thread is not running.
 */
void Thread::wait()
{
	if (thread_.joinable())
		thread_.join();
}

/**
 * \brief Check if the thread is running
 *
 * A Thread instance is considered as running once the underlying thread has
 * started. This method guarantees that it returns true after the start()
 * method returns, and false after the wait() method returns.
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
 * \return The Thread instance for the current thread
 */
Thread *Thread::current()
{
	ThreadData *data = ThreadData::current();
	return data->thread_;
}

/**
 * \brief Set the event dispatcher
 * \param[in] dispatcher Pointer to the event dispatcher
 *
 * Threads that run an event loop require an event dispatcher to integrate
 * event notification and timers with the loop. Users that want to provide
 * their own event dispatcher shall call this method once and only once before
 * the thread is started with start(). If no event dispatcher is provided, a
 * default poll-based implementation will be used.
 *
 * The Thread takes ownership of the event dispatcher and will delete it when
 * the thread is destroyed.
 */
void Thread::setEventDispatcher(std::unique_ptr<EventDispatcher> dispatcher)
{
	if (data_->dispatcher_.load(std::memory_order_relaxed)) {
		LOG(Thread, Warning) << "Event dispatcher is already set";
		return;
	}

	data_->dispatcher_.store(dispatcher.release(),
				 std::memory_order_relaxed);
}

/**
 * \brief Retrieve the event dispatcher
 *
 * This method retrieves the event dispatcher set with setEventDispatcher().
 * If no dispatcher has been set, a default poll-based implementation is created
 * and returned, and no custom event dispatcher may be installed anymore.
 *
 * The returned event dispatcher is valid until the thread is destroyed.
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

}; /* namespace libcamera */
