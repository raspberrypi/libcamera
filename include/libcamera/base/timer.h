/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * timer.h - Generic timer
 */

#pragma once

#include <chrono>
#include <stdint.h>

#include <libcamera/base/private.h>

#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>

namespace libcamera {

class Message;

class Timer : public Object
{
public:
	Timer(Object *parent = nullptr);
	~Timer();

	void start(std::chrono::milliseconds duration);
	void start(std::chrono::steady_clock::time_point deadline);
	void stop();
	bool isRunning() const;

	std::chrono::steady_clock::time_point deadline() const { return deadline_; }

	Signal<> timeout;

protected:
	void message(Message *msg) override;

private:
	void registerTimer();
	void unregisterTimer();

	bool running_;
	std::chrono::steady_clock::time_point deadline_;
};

} /* namespace libcamera */
