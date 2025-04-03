/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Base object
 */

#include <libcamera/base/object.h>

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/base/message.h>
#include <libcamera/base/semaphore.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/utils.h>

/**
 * \file base/object.h
 * \brief Base object to support automatic signal disconnection
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Object)

/**
 * \class Object
 * \brief Base object to support automatic signal disconnection
 *
 * The Object class simplifies signal/slot handling for classes implementing
 * slots. By inheriting from Object, an object is automatically disconnected
 * from all connected signals when it gets destroyed.
 *
 * Object instances are bound to the thread of their parent, or the thread in
 * which they're created when they have no parent. When a message is posted to
 * an object, its handler will run in the object's thread. This allows
 * implementing easy message passing between threads by inheriting from the
 * Object class.
 *
 * Deleting an object from a thread other than the one the object is bound to is
 * unsafe, unless the caller ensures that the object's thread is stopped and no
 * parent or child of the object gets deleted concurrently. See
 * Object::~Object() for more information.
 *
 * Object slots connected to signals will also run in the context of the
 * object's thread, regardless of whether the signal is emitted in the same or
 * in another thread.
 *
 * Objects can be connected to multiple signals, but they can only be connected
 * to each signal once. Attempting to create multiple concurrent connections
 * between the same signal and the same Object (to either the same or differents
 * slots of the object) will cause an assertion failure. While it would be
 * possible to allow the implementation to let objects connect to the same
 * signal multiple times, there are no expected use cases for this in libcamera
 * and this behaviour is restricted to favour defensive programming.
 *
 * \sa Message, Signal, Thread
 */

/**
 * \brief Construct an Object instance
 * \param[in] parent The object parent
 *
 * The new Object instance is bound to the thread of its \a parent, or to the
 * current thread if the \a parent is nullptr.
 */
Object::Object(Object *parent)
	: parent_(parent), pendingMessages_(0)
{
	thread_ = parent ? parent->thread() : Thread::current();

	if (parent)
		parent->children_.push_back(this);
}

/**
 * \brief Destroy an Object instance
 *
 * Deleting an Object automatically disconnects all signals from the Object's
 * slots. All the Object's children are made orphan, but stay bound to their
 * current thread.
 *
 * Object instances shall be destroyed from the thread they are bound to,
 * otherwise undefined behaviour may occur. If deletion of an Object needs to
 * be scheduled from a different thread, deleteLater() shall be used.
 *
 * As an exception to this rule, Object instances may be deleted from a
 * different thread if the thread the instance is bound to is stopped through
 * the whole duration of the object's destruction, *and* the parent and children
 * of the object do not get deleted concurrently. The caller is responsible for
 * fulfilling those requirements.
 *
 * In all cases Object instances shall be deleted before the Thread they are
 * bound to.
 */
Object::~Object()
{
	ASSERT(Thread::current() == thread_ || !thread_->isRunning());

	/*
	 * Move signals to a private list to avoid concurrent iteration and
	 * deletion of items from Signal::disconnect().
	 */
	std::list<SignalBase *> signals(std::move(signals_));
	for (SignalBase *signal : signals)
		signal->disconnect(this);

	if (pendingMessages_)
		thread()->removeMessages(this);

	if (parent_) {
		auto it = std::find(parent_->children_.begin(),
				    parent_->children_.end(), this);
		ASSERT(it != parent_->children_.end());
		parent_->children_.erase(it);
	}

	for (auto child : children_)
		child->parent_ = nullptr;
}

/**
 * \brief Schedule deletion of the instance in the thread it belongs to
 *
 * This function schedules deletion of the Object when control returns to the
 * event loop that the object belongs to. This ensures the object is destroyed
 * from the right context, as required by the libcamera threading model.
 *
 * If this function is called before the thread's event loop is started or after
 * it has stopped, the object will be deleted when the event loop (re)starts. If
 * this never occurs, the object will be leaked.
 *
 * Deferred deletion can be used to control the destruction context with shared
 * pointers. An object managed with shared pointers is deleted when the last
 * reference is destroyed, which makes difficult to ensure through software
 * design which context the deletion will take place in. With a custom deleter
 * for the shared pointer using deleteLater(), the deletion can be guaranteed to
 * happen in the thread the object is bound to.
 *
 * \code{.cpp}
 * std::shared_ptr<MyObject> createObject()
 * {
 *     struct Deleter : std::default_delete<MyObject> {
 *             void operator()(MyObject *obj)
 *             {
 *                     obj->deleteLater();
 *             }
 *     };
 *
 *     MyObject *obj = new MyObject();
 *
 *     return std::shared_ptr<MyObject>(obj, Deleter());
 * }
 * \endcode
 *
 * \context This function is \threadsafe.
 */
void Object::deleteLater()
{
	postMessage(std::make_unique<Message>(Message::DeferredDelete));
}

/**
 * \brief Post a message to the object's thread
 * \param[in] msg The message
 *
 * This function posts the message \a msg to the message queue of the object's
 * thread, to be delivered to the object through the message() function in the
 * context of its thread. Message ownership is passed to the thread, and the
 * message will be deleted after being delivered.
 *
 * Messages are delivered through the thread's event loop. If the thread is not
 * running its event loop the message will not be delivered until the event
 * loop gets started.
 *
 * Due to their asynchronous nature, threads do not provide any guarantee that
 * all posted messages are delivered before the thread is stopped. See
 * \ref thread-stop for additional information.
 *
 * \context This function is \threadsafe.
 */
void Object::postMessage(std::unique_ptr<Message> msg)
{
	thread()->postMessage(std::move(msg), this);
}

/**
 * \brief Message handler for the object
 * \param[in] msg The message
 *
 * This virtual function receives messages for the object. It is called in the
 * context of the object's thread, and can be overridden to process custom
 * messages. The parent Object::message() function shall be called for any
 * message not handled by the override function.
 *
 * The message \a msg is valid only for the duration of the call, no reference
 * to it shall be kept after this function returns.
 */
void Object::message(Message *msg)
{
	switch (msg->type()) {
	case Message::InvokeMessage: {
		/*
		 * A static_cast should be enough, but gcc 10 and 11 choke on
		 * it in release mode (with -O2 or -O3).
		 */
		InvokeMessage *iMsg = dynamic_cast<InvokeMessage *>(msg);
		Semaphore *semaphore = iMsg->semaphore();
		iMsg->invoke();

		if (semaphore)
			semaphore->release();

		break;
	}

	case Message::DeferredDelete:
		delete this;
		break;

	default:
		break;
	}
}

/**
 * \fn Object::assertThreadBound()
 * \brief Check if the caller complies with thread-bound constraints
 * \param[in] message The message to be printed on error
 *
 * This function verifies the calling constraints required by the \threadbound
 * definition. It shall be called at the beginning of member functions of an
 * Object subclass that are explicitly marked as thread-bound in their
 * documentation.
 *
 * If the thread-bound constraints are not met, the function prints \a message
 * as an error message. For debug builds, it additionally causes an assertion
 * error.
 *
 * \todo Verify the thread-bound requirements for functions marked as
 * thread-bound at the class level.
 *
 * \return True if the call is thread-bound compliant, false otherwise
 */
bool Object::assertThreadBound(const char *message)
{
	if (Thread::current() == thread_)
		return true;

	LOG(Object, Error) << message;
	ASSERT(false);
	return false;
}

/**
 * \fn R Object::invokeMethod()
 * \brief Invoke a method asynchronously on an Object instance
 * \param[in] func The object method to invoke
 * \param[in] type Connection type for method invocation
 * \param[in] args The method arguments
 *
 * This function invokes the member method \a func with arguments \a args, based
 * on the connection \a type. Depending on the type, the method will be called
 * synchronously in the same thread or asynchronously in the object's thread.
 *
 * Arguments \a args passed by value or reference are copied, while pointers
 * are passed untouched. The caller shall ensure that any pointer argument
 * remains valid until the method is invoked.
 *
 * Due to the asynchronous nature of threads, functions invoked asynchronously
 * with the ConnectionTypeQueued type are not guaranteed to be called before
 * the thread is stopped. See \ref thread-stop for additional information.
 *
 * \context This function is \threadsafe.
 *
 * \return For connection types ConnectionTypeDirect and
 * ConnectionTypeBlocking, return the return value of the invoked method. For
 * connection type ConnectionTypeQueued, return a default-constructed R value.
 */

/**
 * \fn Object::thread()
 * \brief Retrieve the thread the object is bound to
 * \context This function is \threadsafe.
 * \return The thread the object is bound to
 */

/**
 * \brief Move the object and all its children to a different thread
 * \param[in] thread The target thread
 *
 * This function moves the object and all its children from the current thread
 * to the new \a thread.
 *
 * Before the object is moved, a Message::ThreadMoveMessage message is sent to
 * it. The message() function can be reimplement in derived classes to be
 * notified of the upcoming thread move and perform any required processing.
 *
 * Moving an object that has a parent is not allowed, and causes undefined
 * behaviour.
 *
 * \context This function is \threadbound.
 */
void Object::moveToThread(Thread *thread)
{
	if (!assertThreadBound("Object can't be moved from another thread"))
		return;

	if (thread_ == thread)
		return;

	if (parent_) {
		LOG(Object, Error)
			<< "Moving object to thread with a parent is not permitted";
		return;
	}

	notifyThreadMove();

	thread->moveObject(this);
}

void Object::notifyThreadMove()
{
	Message msg(Message::ThreadMoveMessage);
	message(&msg);

	for (auto child : children_)
		child->notifyThreadMove();
}

/**
 * \fn Object::parent()
 * \brief Retrieve the object's parent
 * \return The object's parent
 */

void Object::connect(SignalBase *signal)
{
	/*
	 * Connecting the same signal to an object multiple times is not
	 * supported.
	 */
	ASSERT(std::find(signals_.begin(), signals_.end(), signal) == signals_.end());

	signals_.push_back(signal);
}

void Object::disconnect(SignalBase *signal)
{
	for (auto iter = signals_.begin(); iter != signals_.end();) {
		if (*iter == signal)
			iter = signals_.erase(iter);
		else
			iter++;
	}
}

} /* namespace libcamera */
