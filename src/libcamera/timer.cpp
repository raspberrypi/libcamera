/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer.cpp - Generic timer
 */

#include <libcamera/timer.h>

#include <time.h>

#include <libcamera/camera_manager.h>
#include <libcamera/event_dispatcher.h>

#include "log.h"

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
 */
Timer::Timer()
	: interval_(0), deadline_(0)
{
}

/**
 * \brief Start or restart the timer with a timeout of \a msec
 * \param[in] msec The timer duration in milliseconds
 *
 * If the timer is already running it will be stopped and restarted.
 */
void Timer::start(unsigned int msec)
{
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);

	interval_ = msec;
	deadline_ = tp.tv_sec * 1000000000ULL + tp.tv_nsec + msec * 1000000ULL;

	LOG(Timer, Debug)
		<< "Starting timer " << this << " with interval "
		<< msec << ": deadline " << deadline_;

	CameraManager::instance()->eventDispatcher()->registerTimer(this);
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
	CameraManager::instance()->eventDispatcher()->unregisterTimer(this);

	deadline_ = 0;
}

/**
 * \brief Check if the timer is running
 * \return True if the timer is running, false otherwise
 */
bool Timer::isRunning() const
{
	return deadline_ != 0;
}

/**
 * \fn Timer::interval()
 * \brief Retrieve the timer interval
 * \return The timer interval in milliseconds
 */

/**
 * \fn Timer::deadline()
 * \brief Retrieve the timer deadline
 * \return The timer deadline in nanoseconds
 */

/**
 * \var Timer::timeout
 * \brief Signal emitted when the timer times out
 *
 * The timer pointer is passed as a parameter.
 */

} /* namespace libcamera */
