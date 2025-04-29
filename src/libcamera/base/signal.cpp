/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Signal & slot implementation
 */

#include <libcamera/base/signal.h>

#include <libcamera/base/mutex.h>
#include <libcamera/base/object.h>

/**
 * \file base/signal.h
 * \brief Signal & slot implementation
 */

namespace libcamera {

namespace {

/*
 * Mutex to protect the SignalBase::slots_ and Object::signals_ lists. If lock
 * contention needs to be decreased, this could be replaced with locks in
 * Object and SignalBase, or with a mutex pool.
 */
Mutex signalsLock;

} /* namespace */

void SignalBase::connect(BoundMethodBase *slot)
{
	MutexLocker locker(signalsLock);

	Object *object = slot->object();
	if (object)
		object->connect(this);
	slots_.push_back(slot);
}

void SignalBase::disconnect(Object *object)
{
	disconnect([object](SlotList::iterator &iter) {
		return (*iter)->match(object);
	});
}

void SignalBase::disconnect(std::function<bool(SlotList::iterator &)> match)
{
	MutexLocker locker(signalsLock);

	for (auto iter = slots_.begin(); iter != slots_.end();) {
		if (match(iter)) {
			Object *object = (*iter)->object();
			if (object)
				object->disconnect(this);

			delete *iter;
			iter = slots_.erase(iter);
		} else {
			++iter;
		}
	}
}

SignalBase::SlotList SignalBase::slots()
{
	MutexLocker locker(signalsLock);
	return slots_;
}

/**
 * \class Signal
 * \brief Generic signal and slot communication mechanism
 *
 * Signals and slots are a language construct aimed at communication between
 * objects through the observer pattern without the need for boilerplate code.
 * See http://doc.qt.io/qt-6/signalsandslots.html for more information.
 *
 * Signals model events that can be observed from objects unrelated to the event
 * source. Slots are functions that are called in response to a signal. Signals
 * can be connected to and disconnected from slots dynamically at runtime. When
 * a signal is emitted, all connected slots are called sequentially in the order
 * they have been connected.
 *
 * Signals are defined with zero, one or more typed parameters. They are emitted
 * with a value for each of the parameters, and those values are passed to the
 * connected slots.
 *
 * Slots are normal static or class member functions. In order to be connected
 * to a signal, their signature must match the signal type (taking the same
 * arguments as the signal and returning void).
 *
 * Connecting a signal to a slot results in the slot being called with the
 * arguments passed to the emit() function when the signal is emitted. Multiple
 * slots can be connected to the same signal, and multiple signals can connected
 * to the same slot.
 *
 * When a slot belongs to an instance of the Object class, the slot is called
 * in the context of the thread that the object is bound to. If the signal is
 * emitted from the same thread, the slot will be called synchronously, before
 * Signal::emit() returns. If the signal is emitted from a different thread,
 * the slot will be called asynchronously from the object's thread's event
 * loop, after the Signal::emit() function returns, with a copy of the signal's
 * arguments. The emitter shall thus ensure that any pointer or reference
 * passed through the signal will remain valid after the signal is emitted.
 *
 * Duplicate connections between a signal and a slot are not expected and use of
 * the Object class to manage signals will enforce this restriction.
 */

/**
 * \fn Signal::connect(T *object, R (T::*func)(Args...))
 * \brief Connect the signal to a member function slot
 * \param[in] object The slot object pointer
 * \param[in] func The slot member function
 *
 * If the typename T inherits from Object, the signal will be automatically
 * disconnected from the \a func slot of \a object when \a object is destroyed.
 * Otherwise the caller shall disconnect signals manually before destroying \a
 * object.
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::connect(T *object, Func func)
 * \brief Connect the signal to a function object slot
 * \param[in] object The slot object pointer
 * \param[in] func The function object
 *
 * If the typename T inherits from Object, the signal will be automatically
 * disconnected from the \a func slot of \a object when \a object is destroyed.
 * Otherwise the caller shall disconnect signals manually before destroying \a
 * object.
 *
 * The function object is typically a lambda function, but may be any object
 * that satisfies the FunctionObject named requirements. The types of the
 * function object arguments shall match the types of the signal arguments.
 *
 * No matching disconnect() function exist, as it wouldn't be possible to pass
 * to a disconnect() function the same lambda that was passed to connect(). The
 * connection created by this function can not be removed selectively if the
 * signal is connected to multiple slots of the same receiver, but may be
 * otherwise be removed using the disconnect(T *object) function.
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::connect(R (*func)(Args...))
 * \brief Connect the signal to a static function slot
 * \param[in] func The slot static function
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::disconnect()
 * \brief Disconnect the signal from all slots
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::disconnect(T *object)
 * \brief Disconnect the signal from all slots of the \a object
 * \param[in] object The object pointer whose slots to disconnect
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::disconnect(T *object, R (T::*func)(Args...))
 * \brief Disconnect the signal from the \a object slot member function \a func
 * \param[in] object The object pointer whose slots to disconnect
 * \param[in] func The slot member function to disconnect
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::disconnect(R (*func)(Args...))
 * \brief Disconnect the signal from the slot static function \a func
 * \param[in] func The slot static function to disconnect
 *
 * \context This function is \threadsafe.
 */

/**
 * \fn Signal::emit(Args... args)
 * \brief Emit the signal and call all connected slots
 * \param args The arguments passed to the connected slots
 *
 * Emitting a signal calls all connected slots synchronously and sequentially in
 * the order the slots have been connected. The arguments passed to the emit()
 * function are passed to the slot functions unchanged. If a slot modifies one
 * of the arguments (when passed by pointer or reference), the modification is
 * thus visible to all subsequently called slots.
 *
 * This function is not \threadsafe, but thread-safety is guaranteed against
 * concurrent connect() and disconnect() calls.
 */

} /* namespace libcamera */
