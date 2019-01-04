/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer.h - Generic timer
 */
#ifndef __LIBCAMERA_TIMER_H__
#define __LIBCAMERA_TIMER_H__

#include <cstdint>

#include <libcamera/signal.h>

namespace libcamera {

class Timer
{
public:
	Timer();

	void start(unsigned int msec);
	void stop();
	bool isRunning() const;

	unsigned int interval() const { return interval_; }
	uint64_t deadline() const { return deadline_; }

	Signal<Timer *> timeout;

private:
	unsigned int interval_;
	uint64_t deadline_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_TIMER_H__ */
