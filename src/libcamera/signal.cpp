/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * signal.cpp - Signal & slot implementation
 */

#include <libcamera/signal.h>

/**
 * \file signal.h
 * \brief Signal & slot implementation
 */

namespace libcamera {

/**
 * \class Signal
 * \brief Generic signal and slot communication mechanism
 *
 * Signals and slots are a language construct aimed at communication between
 * objects through the observer pattern without the need for boilerplate code.
 * See http://doc.qt.io/qt-5/signalsandslots.html for more information.
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
 * to the same slot. Duplicate connections between a signal and a slot are
 * allowed and result in the slot being called multiple times for the same
 * signal emission.
 */

/**
 * \fn Signal::connect(T *object, void(T::*func)(Args...))
 * \brief Connect the signal to a member function slot
 * \param[in] object The slot object pointer
 * \param[in] func The slot member function
 *
 * If the typename T inherits from Object, the signal will be automatically
 * disconnected from the \a func slot of \a object when \a object is destroyed.
 * Otherwise the caller shall disconnect signals manually before destroying \a
 * object.
 */

/**
 * \fn Signal::connect(void(*func)(Args...))
 * \brief Connect the signal to a static function slot
 * \param[in] func The slot static function
 */

/**
 * \fn Signal::disconnect()
 * \brief Disconnect the signal from all slots
 */

/**
 * \fn Signal::disconnect(T *object)
 * \brief Disconnect the signal from all slots of the \a object
 * \param[in] object The object pointer whose slots to disconnect
 */

/**
 * \fn Signal::disconnect(T *object, void(T::*func)(Args...))
 * \brief Disconnect the signal from the \a object slot member function \a func
 * \param[in] object The object pointer whose slots to disconnect
 * \param[in] func The slot member function to disconnect
 */

/**
 * \fn Signal::disconnect(void(*func)(Args...))
 * \brief Disconnect the signal from the slot static function \a func
 * \param[in] func The slot static function to disconnect
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
 */

} /* namespace libcamera */
