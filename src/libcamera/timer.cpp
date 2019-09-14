/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer.cpp - Generic timer
 */

#include <libcamera/timer.h>

#include <chrono>

#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>

#include "log.h"
#include "message.h"
#include "thread.h"
#include "utils.h"

/**
 * \file timer.h
 * \brief Generic timer
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Timer)

/**
 * \class Timer
 * \brief Single-shot timer interface
 *
 * The Timer class models a single-shot timer that is started with start() and
 * emits the \ref timeout signal when it times out.
 *
 * Once started the timer will run until it times out. It can be stopped with
 * stop(), and once it times out or is stopped, can be started again with
 * start().
 */

/**
 * \brief Construct a timer
 * \param[in] parent The parent Object
 */
Timer::Timer(Object *parent)
	: Object(parent)
{
}

Timer::~Timer()
{
	stop();
}

/**
 * \fn Timer::start(unsigned int msec)
 * \brief Start or restart the timer with a timeout of \a msec
 * \param[in] msec The timer duration in milliseconds
 *
 * If the timer is already running it will be stopped and restarted.
 */

/**
 * \brief Start or restart the timer with a timeout of \a interval
 * \param[in] interval The timer duration in milliseconds
 *
 * If the timer is already running it will be stopped and restarted.
 */
void Timer::start(std::chrono::milliseconds interval)
{
	interval_ = interval;
	deadline_ = utils::clock::now() + interval;

	LOG(Timer, Debug)
		<< "Starting timer " << this << " with interval "
		<< interval.count() << ": deadline "
		<< utils::time_point_to_string(deadline_);

	registerTimer();
}

/**
 * \brief Stop the timer
 *
 * After this function returns the timer is guaranteed not to emit the
 * \ref timeout signal.
 *
 * If the timer is not running this function performs no operation.
 */
void Timer::stop()
{
	unregisterTimer();

	deadline_ = utils::time_point();
}

void Timer::registerTimer()
{
	thread()->eventDispatcher()->registerTimer(this);
}

void Timer::unregisterTimer()
{
	thread()->eventDispatcher()->unregisterTimer(this);
}

/**
 * \brief Check if the timer is running
 * \return True if the timer is running, false otherwise
 */
bool Timer::isRunning() const
{
	return deadline_ != utils::time_point();
}

/**
 * \fn Timer::interval()
 * \brief Retrieve the timer interval
 * \return The timer interval in milliseconds
 */

/**
 * \fn Timer::deadline()
 * \brief Retrieve the timer deadline
 * \return The timer deadline
 */

/**
 * \var Timer::timeout
 * \brief Signal emitted when the timer times out
 *
 * The timer pointer is passed as a parameter.
 */

void Timer::message(Message *msg)
{
	if (msg->type() == Message::ThreadMoveMessage) {
		if (isRunning()) {
			unregisterTimer();
			invokeMethod(&Timer::registerTimer);
		}
	}

	Object::message(msg);
}

} /* namespace libcamera */
